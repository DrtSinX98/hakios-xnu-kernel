/*
 * The realme 8i's touch screen: the touch half of the ILI7807S (display and touch in one chip) on SPI4 of the MT6781,
 * as a source of touches for /dev/hakios-touch (sys/hakios.input.h), the same path the Hot 20's hakios.tcm.c feeds.
 * Boot-arg touchscreen=3; touchlog=1 logs every finger's landing and lifting and every eighth move.
 *
 * The chip keeps its firmware in RAM, and LK holds it in reset: mtkloader (XNU_TOUCH_ILI=1, mtk/loader/mtkili.h)
 * lets it out and sends the firmware on the way here, so by now it reports. All that is left is reading:
 *   pins     GPIO13 data out, 14 data in, 144 clock, 145 chip select, driven by hand (mode 0, MSB first): SPI4's
 *            controller hung in mtkloader on anything but single short transfers, and a report is 46 bytes
 *   INT      GPIO12 / EINT 12 gives a short low pulse per report. It is not taken as an interrupt: EINT 12 is set
 *            to falling edge and unmasked, so that the edge is latched in EINT_STA (masked, it latched nothing in
 *            the first run, and 4 ms looks at the line caught 4 reports in 2.5 minutes), while the EINT block's
 *            own line into the GIC (SPI 186) is switched off at the distributor; a thread looks at the latch (and
 *            at the line itself) every touch_poll_us (4000) and reads one report when either says so
 *   lifts    a finger with no report for 150 ms is lifted: one missed last report must not become a long press
 *            (the first run's fingers stayed down for seconds and SpringBoard went into its editing mode)
 *   report   0x83, then 45 bytes back: two, the firmware's ACK 0xA3, and the 43-byte demo packet: 0x5A, ten
 *            fingers of four bytes (x high nibble | y high nibble, x low, y low, pressure; FF FF FF: none), and a
 *            checksum (minus the sum of the rest)
 *   place    0..2047 on the chip's grid; x * 1079 / 2047 and y * 2411 / 2047 on the 1080 x 2412 panel, then
 *            0..65535 across it for /dev/hakios-touch
 *   reload   the chip loses the firmware by itself (three runs: gone by the first read, the reset lines untouched),
 *            so when it answers without its ACK the firmware is sent again, as Android's driver does at every display
 *            resume and on its ESD check, and as mtkloader sends it: ICE mode with the MCU stopped, the blocks AP, DATA
 *            and TUNING, each checked against the chip's DMA CRC and its last four bytes, the code reset, ICE mode left.
 *            The firmware is mtkloader's own copy, still in memory under XNU: its table's address comes as ili_fw=
 *            (each block's CRC is checked here before it is used). At most every 2 s, and given up on after five
 *            failures in a row.
 *   edge     after each load the firmware is told to reject palms at the edges (edge_palm on, 01 12 01, as Android's
 *            driver tells it at every resume): without it a hand holding the phone was a second finger at x = 0
 * Besides those the only thing sent to the chip is the read command 0x83. Register layouts from MediaTek's MT6781
 * tables (pinctrl-mtk-mt6781.h, mtk-eint.c); the chip's framing from the ILITEK driver Android runs (ili9882n),
 * read, not copied.
 */
#include <kern/clock.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.input.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern void IOSleep(unsigned int milliseconds);

#define GPIO_PHYS       0x10005000
#define EINT_PHYS       0x1000b000
#define GICD_PHYS       0x0c000000
#define GICD_ISENABLER6 0x118                   /* INTIDs 192..223 */
#define GICD_ICENABLER6 0x198
#define EINT_GIC_BIT    (1u << (218 % 32))      /* SPI 186 (the device tree's pio interrupt 0xba) = INTID 218 */
#define STALE_MS        150
#define PIN_INT         12
#define PIN_MO          13
#define PIN_MI          14
#define PIN_CLK         144
#define PIN_CS          145
#define PIN_RST         121                     /* the chip's reset, active low (mtkloader lets it go) */
#define PIN_LCM_RST     99                      /* the panel's reset */
#define EINT_BIT        (1u << 12)              /* EINT 12, in the first word of each EINT register */
#define EINT_STA        0x000
#define EINT_ACK        0x040
#define EINT_MASK_CLR   0x100
#define EINT_SENS_CLR   0x1c0                   /* clear: edge */
#define EINT_POL_CLR    0x380                   /* clear: falling */
#define EINT_DOM_EN     0x400

#define PANEL_W         1080
#define PANEL_H         2412
#define FINGERS         10
#define PACKET          43
#define BLOCKS          3                       /* AP, DATA, TUNING */
#define CHUNK           2048                    /* bytes of a block in one SPI write */
#define RELOAD_GAP_MS   2000
#define RELOAD_TRIES    5

/* mtkloader's struct ili_block (mtk/tools/mkilifw.py), as it lies in memory */
struct loader_block {
	uint64_t name;
	uint32_t mem, len, crc, pad;
	uint64_t data;
};

static struct {
	uint32_t mem, len, crc;
	const volatile uint8_t *data;
} fw[BLOCKS];
static bool have_fw;
static unsigned reloads, reload_failures;

static vm_offset_t gpio, eint, gicd;
static uint64_t last_report;                    /* mach_absolute_time of the last good report */
static unsigned stale;
static uint32_t log_reports;
static unsigned reports, touches, bad, latched, level;
static bool down[FINGERS];
static uint32_t last_x[FINGERS], last_y[FINGERS];

static inline uint32_t
rd(vm_offset_t base, uint32_t reg)
{
	return *(volatile uint32_t *)(base + reg);
}

static inline void
wr(vm_offset_t base, uint32_t reg, uint32_t value)
{
	*(volatile uint32_t *)(base + reg) = value;
	__asm__ volatile ("dsb sy" ::: "memory");
}

/* DOUT and DIR have set and clear registers at +4 and +8, 32 pins per 0x10; DIN at 0x200; MODE four bits a pin */
static inline void
pin_set(uint32_t pin, int high)
{
	wr(gpio, (high ? 0x104 : 0x108) + 0x10 * (pin / 32), 1u << (pin % 32));
}

static inline int
pin_get(uint32_t pin)
{
	return (rd(gpio, 0x200 + 0x10 * (pin / 32)) >> (pin % 32)) & 1;
}

static void
pin_mode(uint32_t pin, uint32_t func)
{
	uint32_t reg = 0x300 + 0x10 * (pin / 8), shift = (pin % 8) * 4;

	wr(gpio, reg, (rd(gpio, reg) & ~(0xfu << shift)) | (func << shift));
}

/* the reset lines and INT as they are now: whether something has taken the chip down */
static void
say_lines(const char *why, const uint8_t *rx)
{
	uint64_t ns;

	absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
	printf("ili: %s at %llu ms: ACK %02x packet %02x; INT %s; GPIO121 mode %u %s %s; GPIO99 mode %u %s %s\n", why,
	    ns / 1000000, rx[1], rx[3], pin_get(PIN_INT) ? "high" : "low",
	    (rd(gpio, 0x300 + 0x10 * (PIN_RST / 8)) >> ((PIN_RST % 8) * 4)) & 0xf,
	    ((rd(gpio, 0x000 + 0x10 * (PIN_RST / 32)) >> (PIN_RST % 32)) & 1) ? "out" : "in", pin_get(PIN_RST) ? "high" : "low",
	    (rd(gpio, 0x300 + 0x10 * (PIN_LCM_RST / 8)) >> ((PIN_LCM_RST % 8) * 4)) & 0xf,
	    ((rd(gpio, 0x000 + 0x10 * (PIN_LCM_RST / 32)) >> (PIN_LCM_RST % 32)) & 1) ? "out" : "in",
	    pin_get(PIN_LCM_RST) ? "high" : "low");
}

static void
pins_by_hand(void)
{
	pin_set(PIN_CS, 1);                             /* levels first, then outputs, then the pins taken from SPI4 */
	pin_set(PIN_CLK, 0);
	pin_set(PIN_MO, 0);
	wr(gpio, 0x004 + 0x10 * (PIN_CS / 32), 1u << (PIN_CS % 32));
	wr(gpio, 0x004 + 0x10 * (PIN_CLK / 32), 1u << (PIN_CLK % 32));
	wr(gpio, 0x004 + 0x10 * (PIN_MO / 32), 1u << (PIN_MO % 32));
	wr(gpio, 0x008 + 0x10 * (PIN_MI / 32), 1u << (PIN_MI % 32));
	pin_mode(PIN_CS, 0);
	pin_mode(PIN_CLK, 0);
	pin_mode(PIN_MO, 0);
	pin_mode(PIN_MI, 0);
}

/* one transfer, chip select held throughout: tx_len bytes of tx, then zeros, len bytes in all; rx NULL keeps nothing */
static void
spi_xfer(const uint8_t *tx, uint32_t tx_len, uint8_t *rx, uint32_t len)
{
	pin_set(PIN_CS, 0);
	(void)pin_get(PIN_MI);
	for (uint32_t i = 0; i < len; i++) {
		uint8_t out = i < tx_len ? tx[i] : 0, in = 0;

		for (int bit = 7; bit >= 0; bit--) {
			pin_set(PIN_MO, (out >> bit) & 1);
			pin_set(PIN_CLK, 1);
			in = (uint8_t)(in << 1 | pin_get(PIN_MI));
			pin_set(PIN_CLK, 0);
			(void)pin_get(PIN_MI);
		}
		if (rx) {
			rx[i] = in;
		}
	}
	pin_set(PIN_CS, 1);
	(void)pin_get(PIN_MI);
}

static void
spi_read(uint8_t *rx, uint32_t len)
{
	static const uint8_t read_cmd = 0x83;

	spi_xfer(&read_cmd, 1, rx, len);
}

/* ---- sending the firmware again ---- */

static uint32_t
crc32_msb(const volatile uint8_t *p, uint32_t len)
{
	uint32_t crc = 0xffffffff;

	for (uint32_t i = 0; i < len; i++) {
		crc ^= (uint32_t)p[i] << 24;
		for (int b = 0; b < 8; b++) {
			crc = (crc & 0x80000000) ? (crc << 1) ^ 0x04c11db7 : crc << 1;
		}
	}
	return crc;
}

/* mtkloader's copy of the firmware, found and checked */
static void
find_fw(void)
{
	uint64_t table = 0;
	const volatile struct loader_block *b;

	if (!PE_parse_boot_argn("ili_fw", &table, sizeof(table)) || table == 0) {
		printf("ili: no ili_fw= from mtkloader: the firmware cannot be sent again\n");
		return;
	}
	b = (const volatile struct loader_block *)(ml_io_map(trunc_page(table), 2 * PAGE_SIZE) + (table & PAGE_MASK));
	for (int i = 0; i < BLOCKS; i++) {
		uint64_t pa = b[i].data;
		uint32_t len = b[i].len, got;

		if (len < 8 || len > (160u << 10) || b[i].mem >= 0x30000 || pa == 0) {
			printf("ili: mtkloader's firmware table at %#llx does not look like one (block %d: %#x, %u bytes): not used\n",
			    table, i, b[i].mem, len);
			return;
		}
		fw[i].mem = b[i].mem;
		fw[i].len = len;
		fw[i].crc = b[i].crc;
		fw[i].data = (const volatile uint8_t *)(ml_io_map(trunc_page(pa), round_page((pa & PAGE_MASK) + len)) + (pa & PAGE_MASK));
		got = crc32_msb(fw[i].data, len - 4);
		if (got != fw[i].crc) {
			printf("ili: mtkloader's firmware block %d has CRC %08x, not %08x: not used\n", i, got, fw[i].crc);
			return;
		}
	}
	have_fw = true;
	printf("ili: mtkloader's firmware found at %#llx: %u + %u + %u bytes, CRCs %08x %08x %08x as they should be\n", table,
	    fw[0].len, fw[1].len, fw[2].len, fw[0].crc, fw[1].crc, fw[2].crc);
}

static void
ice_write(uint32_t addr, uint32_t value, uint32_t n)
{
	uint8_t w[9] = { 0x82, 0x25, (uint8_t)addr, (uint8_t)(addr >> 8), (uint8_t)(addr >> 16) };

	for (uint32_t i = 0; i < n; i++) {
		w[5 + i] = (uint8_t)(value >> (8 * i));
	}
	spi_xfer(w, 5 + n, NULL, 5 + n);
}

/* n (<= 4) bytes of the chip's memory, in the order they come */
static void
ice_bytes(uint32_t addr, uint8_t *b, uint32_t n)
{
	uint8_t w[5] = { 0x82, 0x25, (uint8_t)addr, (uint8_t)(addr >> 8), (uint8_t)(addr >> 16) }, in[5];

	spi_xfer(w, 5, NULL, 5);
	spi_read(in, 1 + n);
	for (uint32_t i = 0; i < n; i++) {
		b[i] = in[1 + i];
	}
}

/* the chip's DMA CRC over [start, start + size), the driver's calc_hw_dma_crc; 0 when it does not finish */
static uint32_t
dma_crc(uint32_t start, uint32_t size)
{
	uint8_t b[4] = { 0 };

	ice_write(0x072104, start, 4);
	ice_write(0x072108, 0x80000001, 4);
	ice_write(0x072114, 0x0002725c, 4);
	ice_write(0x072118, 0x80000000, 4);
	ice_write(0x07211c, size, 4);
	ice_write(0x041016, 0x00, 1);
	ice_write(0x041017, 0x03, 1);
	ice_write(0x072100, 0x02000000, 4);
	ice_write(0x041016, 0x01, 1);
	ice_write(0x048006, 0x2, 1);
	ice_write(0x072100, 0x01000000, 4);
	for (int i = 0; i < 50; i++) {
		IOSleep(1);
		ice_bytes(0x048006, b, 1);
		if (b[0] & 2) {
			ice_bytes(0x04101c, b, 4);
			return (uint32_t)b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16 | (uint32_t)b[3] << 24;
		}
	}
	return 0;
}

/* the firmware's edge-palm rejection on: a command, framed 0x82, length (two bytes), then 01 12 01 */
static void
edge_palm_on(void)
{
	static const uint8_t cmd[6] = { 0x82, 0x00, 0x03, 0x01, 0x12, 0x01 };

	spi_xfer(cmd, sizeof(cmd), NULL, sizeof(cmd));
	printf("ili: edge-palm rejection on\n");
}

static bool
reload(void)
{
	static const uint8_t wake[12] = { 0x82, 0x00, 0x09, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3 };
	static const uint8_t ice_stop[5] = { 0x82, 0x25, 0x62, 0x10, 0x18 };
	static const uint8_t ice_off[5] = { 0x82, 0x1b, 0x62, 0x10, 0x18 };
	static uint8_t buf[5 + CHUNK + 4];
	uint32_t dma[BLOCKS] = { 0 }, iram[BLOCKS] = { 0 };
	bool good = true;

	spi_xfer(wake, sizeof(wake), NULL, sizeof(wake));
	spi_xfer(ice_stop, sizeof(ice_stop), NULL, sizeof(ice_stop));
	IOSleep(1);
	for (int i = 0; i < BLOCKS && good; i++) {
		uint8_t b[4];

		for (uint32_t at = 0; at < fw[i].len; at += CHUNK) {
			uint32_t n = fw[i].len - at < CHUNK ? fw[i].len - at : CHUNK, a = fw[i].mem + at;

			buf[0] = 0x82;
			buf[1] = 0x25;
			buf[2] = (uint8_t)a;
			buf[3] = (uint8_t)(a >> 8);
			buf[4] = (uint8_t)(a >> 16);
			for (uint32_t k = 0; k < n; k++) {
				buf[5 + k] = fw[i].data[at + k];
			}
			while (n % 4) {
				buf[5 + n++] = 0xff;
			}
			spi_xfer(buf, 5 + n, NULL, 5 + n);
		}
		dma[i] = dma_crc(fw[i].mem, fw[i].len - 4);
		ice_bytes(fw[i].mem + fw[i].len - 4, b, 4);
		iram[i] = (uint32_t)b[0] << 24 | (uint32_t)b[1] << 16 | (uint32_t)b[2] << 8 | b[3];
		good = dma[i] == fw[i].crc && iram[i] == fw[i].crc;
	}
	if (good) {
		ice_write(0x40040, 0xae, 1);                    /* the code reset: the MCU starts the firmware */
	}
	spi_xfer(ice_off, sizeof(ice_off), NULL, sizeof(ice_off));
	IOSleep(100);
	printf("ili: firmware sent again (%s): dma/iram %08x/%08x %08x/%08x %08x/%08x\n", good ? "CRCs right" : "A CRC WRONG",
	    dma[0], iram[0], dma[1], iram[1], dma[2], iram[2]);
	return good;
}

static uint32_t
across(uint32_t v, uint32_t pixels)
{
	return v * 65535 / (pixels - 1);
}

/* a report: each finger it names lands or moves, each one it no longer names lifts */
static void
report(const uint8_t *p)
{
	bool seen[FINGERS] = { false };

	for (uint32_t f = 0; f < FINGERS; f++) {
		const uint8_t *q = p + 1 + 4 * f;
		uint32_t x, y;

		if (q[0] == 0xff && q[1] == 0xff && q[2] == 0xff) {
			continue;
		}
		x = ((uint32_t)(q[0] & 0xf0) << 4 | q[1]) * (PANEL_W - 1) / 2047;
		y = ((uint32_t)(q[0] & 0x0f) << 8 | q[2]) * (PANEL_H - 1) / 2047;
		if (x >= PANEL_W) {
			x = PANEL_W - 1;
		}
		if (y >= PANEL_H) {
			y = PANEL_H - 1;
		}
		if (log_reports && (!down[f] || touches % 8 == 0)) {
			printf("ili: f%u %s %u,%u p%u\n", f, down[f] ? "move" : "down", x, y, q[3]);
		}
		hakios_touch_post(down[f] ? HAKIOS_TOUCH_MOVE : HAKIOS_TOUCH_DOWN, f, across(x, PANEL_W), across(y, PANEL_H));
		down[f] = seen[f] = true;
		last_x[f] = x;
		last_y[f] = y;
		touches++;
	}
	last_report = mach_absolute_time();
	for (uint32_t f = 0; f < FINGERS; f++) {
		if (down[f] && !seen[f]) {
			if (log_reports) {
				printf("ili: f%u up %u,%u\n", f, last_x[f], last_y[f]);
			}
			hakios_touch_post(HAKIOS_TOUCH_UP, f, across(last_x[f], PANEL_W), across(last_y[f], PANEL_H));
			down[f] = false;
		}
	}
	reports++;
}

static void
ili_thread(__unused void *parameter, __unused wait_result_t result)
{
	uint32_t poll_us = 4000;
	uint8_t rx[3 + PACKET];
	bool alive;
	uint64_t last_try = 0;

	PE_parse_boot_argn("touch_poll_us", &poll_us, sizeof(poll_us));
	if (poll_us < 250 || poll_us > 100000) {
		poll_us = 4000;
	}
	gpio = ml_io_map(GPIO_PHYS, 0x1000);
	eint = ml_io_map(EINT_PHYS, 0x1000);
	gicd = ml_io_map(GICD_PHYS, 0x1000);
	pins_by_hand();
	find_fw();
	/* the EINT block's line into the GIC off first, then EINT 12: falling edge, its old latch cleared, unmasked */
	printf("ili: GIC SPI 186 (the EINT block) %s; switched off\n", (rd(gicd, GICD_ISENABLER6) & EINT_GIC_BIT) ? "was on" : "was off");
	wr(gicd, GICD_ICENABLER6, EINT_GIC_BIT);
	wr(eint, EINT_SENS_CLR, EINT_BIT);
	wr(eint, EINT_POL_CLR, EINT_BIT);
	wr(eint, EINT_DOM_EN, rd(eint, EINT_DOM_EN) | EINT_BIT);
	wr(eint, EINT_ACK, EINT_BIT);
	wr(eint, EINT_MASK_CLR, EINT_BIT);
	printf("ili: the touch screen read by hand on SPI4's pins; EINT 12 latched on a falling edge, looked at every %u us%s\n",
	    poll_us, log_reports ? "; touchlog=1: landings, liftings and every eighth move logged" : "");
	/* one read to see whether the firmware mtkloader sent still answers (its ACK 0xA3) */
	spi_read(rx, sizeof(rx));
	say_lines(rx[1] == 0xa3 ? "firmware answers" : "FIRMWARE GONE", rx);
	alive = rx[1] == 0xa3;
	if (alive) {
		edge_palm_on();
	}

	for (unsigned tick = 1;; tick++) {
		bool edge = (rd(eint, EINT_STA) & EINT_BIT) != 0, low = !pin_get(PIN_INT);

		if (edge || low) {
			uint8_t sum = 0;

			if (edge) {
				wr(eint, EINT_ACK, EINT_BIT);
				latched++;
			} else {
				level++;
			}
			spi_read(rx, sizeof(rx));
			if ((rx[1] == 0xa3) != alive) {
				alive = rx[1] == 0xa3;
				say_lines(alive ? "firmware answers again" : "FIRMWARE GONE", rx);
			}
			for (uint32_t i = 0; i < PACKET - 1; i++) {
				sum += rx[3 + i];
			}
			if (rx[1] == 0xa3 && rx[3] == 0x5a && (uint8_t)(sum + rx[3 + PACKET - 1]) == 0) {
				report(rx + 3);
			} else if (bad++ < 20) {
				printf("ili: not a report: ACK %02x, packet %02x, checksum %02x against %02x\n", rx[1], rx[3],
				    (uint8_t)-sum, rx[3 + PACKET - 1]);
			}
		}
		/* the firmware gone: sent again, at most every RELOAD_GAP_MS, not after RELOAD_TRIES failures in a row */
		if (!alive && have_fw && reload_failures < RELOAD_TRIES) {
			uint64_t ns;

			absolutetime_to_nanoseconds(mach_absolute_time() - last_try, &ns);
			if (last_try == 0 || ns > RELOAD_GAP_MS * 1000000ULL) {
				bool sent = reload();

				last_try = mach_absolute_time();
				reloads++;
				spi_read(rx, sizeof(rx));
				alive = rx[1] == 0xa3;
				reload_failures = sent && alive ? 0 : reload_failures + 1;
				say_lines(alive ? "firmware answers after sending it again" : "firmware STILL GONE after sending it again", rx);
				if (alive) {
					edge_palm_on();
				}
				wr(eint, EINT_ACK, EINT_BIT);
				if (reload_failures == RELOAD_TRIES) {
					printf("ili: %u failures in a row: the firmware is not sent again\n", RELOAD_TRIES);
				}
			}
		}
		if (last_report != 0) {
			uint64_t ns;

			absolutetime_to_nanoseconds(mach_absolute_time() - last_report, &ns);
			if (ns > STALE_MS * 1000000ULL) {
				for (uint32_t f = 0; f < FINGERS; f++) {
					if (down[f]) {
						if (log_reports) {
							printf("ili: f%u up %u,%u (no report for %u ms)\n", f, last_x[f], last_y[f], STALE_MS);
						}
						hakios_touch_post(HAKIOS_TOUCH_UP, f, across(last_x[f], PANEL_W), across(last_y[f], PANEL_H));
						down[f] = false;
						stale++;
					}
				}
				last_report = 0;
			}
		}
		if (tick % (30 * (1000000 / poll_us)) == 0) {
			say_lines("now", rx);
			printf("ili: so far %u reports (%u by the EINT latch, %u by the line low), %u finger positions, %u not "
			    "reports, %u fingers lifted for want of a report, firmware sent again %u times\n", reports, latched, level,
			    touches, bad, stale, reloads);
		}
		delay(poll_us);
	}
}

void
hakios_mt6781_ili_init(void)
{
	uint32_t which = 0;
	thread_t thread;

	if (!PE_parse_boot_argn("touchscreen", &which, sizeof(which)) || which != 3) {
		return;
	}
	PE_parse_boot_argn("touchlog", &log_reports, sizeof(log_reports));
	if (kernel_thread_start(ili_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	} else {
		printf("ili: cannot start the thread that reads the touch screen\n");
	}
}
