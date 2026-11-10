/*
 * The realme 8i's touch controller: the touch half of the ILI7807S (display and touch in one chip,
 * no flash of its own), on SPI4 of the MT6781. ILI_PROBE=1: what LK leaves, then one read with
 * nothing written, then the chip's ID through ICE mode with its MCU left running. Then Android.
 *
 * Addresses from the phone's device tree and MediaTek's MT6781 tables (pinctrl-mtk-mt6781.h,
 * clk-mt6781.c); the chip's SPI framing from the ILITEK driver Android runs (ili9882n), read, not
 * copied:
 *   write   0x82, then the bytes (in ICE mode no length follows)
 *   read    0x83, then clocked-out bytes; outside ICE mode two bytes come first and the second is
 *           the firmware's ACK 0xA3 when firmware runs
 *   ICE     0x82 0x1F 0x62 0x10 0x18 enters it with the MCU running (0x25 would stop it),
 *           0x82 0x1B 0x62 0x10 0x18 leaves it; a register: 0x82 0x25 addr[23:0] LE, then 0x83 + 4
 * LK leaves the chip's reset (GPIO121, active low) held low. ILI_RELEASE_RST=1 lets it go (drives it high, then
 * 10 ms, as the end of Android's reset pulse) before the reads; without it the chip cannot answer.
 *
 * SPI4 runs on the clock LK leaves it (spi_sel 3, mainpll_d5_d2, 109.2 MHz): switched to the crystal in the first
 * run its second transfer never finished (STATUS1 busy).
 */
#ifndef MTKILI_H
#define MTKILI_H

#define SPI4_BASE            0x11014000ULL
#define EINT_BASE            0x1000b000ULL
#define ILI_TICKS            9                    /* 109.2 MHz / 18: 6.1 MHz, under the 9.6 the device tree allows */
#define SPI_CMD_FINISH_IE    (1u << 16)
#ifndef ILI_RELEASE_RST
#define ILI_RELEASE_RST      0
#endif
#ifndef ILI_DOWNLOAD
#define ILI_DOWNLOAD         0
#endif
#define INFRA2_CG_CLR        0xa8
#define INFRA2_CG_STA        0xac                 /* a set bit: that clock held off */
#define INFRA2_SPI4          (1u << 25)

#define ILI_PIN_INT          12                   /* EINT 12, falling edge */
#define ILI_PIN_MO           13
#define ILI_PIN_MI           14
#define ILI_PIN_CLK          144
#define ILI_PIN_CS           145
#define ILI_PIN_RST          121
#define LCM_PIN_RST          99
#define ILI_SPI_FUNC         2

static uint32_t gpio_mode_of(uint32_t pin)
{
	return (rd32(GPIO_BASE + 0x300 + 0x10 * (pin / 8)) >> ((pin % 8) * 4)) & 0xf;
}

static void gpio_mode_to(uint32_t pin, uint32_t func)
{
	uint64_t reg = GPIO_BASE + 0x300 + 0x10 * (pin / 8);
	uint32_t shift = (pin % 8) * 4;

	wr32(reg, (rd32(reg) & ~(0xfu << shift)) | (func << shift));
	__asm__ volatile("dsb sy");
}

static uint32_t gpio_bit(uint32_t base_ofs, uint32_t pin)
{
	return (rd32(GPIO_BASE + base_ofs + 0x10 * (pin / 32)) >> (pin % 32)) & 1;
}

/* One transfer on SPI4, mode 0, MSB first, chip select held for all of it; at most the FIFO's 32 bytes. */
static int spi4_xfer(const uint8_t *tx, uint8_t *rx, uint32_t len)
{
	uint32_t words = (len + 3) / 4;

	if (len == 0 || len > SPI_FIFO_BYTES) {
		return 0;
	}
	wr32(SPI4_BASE + SPI_CMD, SPI_CMD_RST);
	wr32(SPI4_BASE + SPI_CMD, 0);
	wr32(SPI4_BASE + SPI_CFG2, ((ILI_TICKS - 1) << SPI_CFG2_SCK_HIGH_SHIFT) | ((ILI_TICKS - 1) << SPI_CFG2_SCK_LOW_SHIFT));
	wr32(SPI4_BASE + SPI_CFG0, ((ILI_TICKS - 1) << SPI_CFG0_CS_HOLD_SHIFT) | ((ILI_TICKS - 1) << SPI_CFG0_CS_SETUP_SHIFT));
	wr32(SPI4_BASE + SPI_PAD_SEL, 0);               /* mediatek,pad-select = <0> */
	wr32(SPI4_BASE + SPI_CMD, SPI_CMD_TXMSBF | SPI_CMD_RXMSBF | SPI_CMD_FINISH_IE);  /* (as Linux; the GIC line stays off) */
	wr32(SPI4_BASE + SPI_CFG1, ((len - 1) << SPI_CFG1_PACKET_LEN_SHIFT) | ((ILI_TICKS - 1) << SPI_CFG1_CS_IDLE_SHIFT));
	for (uint32_t w = 0; w < words; w++) {
		uint32_t v = 0;

		for (uint32_t b = 0; b < 4; b++) {
			uint32_t i = w * 4 + b;

			v |= (uint32_t)(tx && i < len ? tx[i] : 0) << (8 * b);
		}
		wr32(SPI4_BASE + SPI_TX_DATA, v);
	}
	(void)rd32(SPI4_BASE + SPI_STATUS0);           /* read clears it: nothing left from before */
	wr32(SPI4_BASE + SPI_CMD, rd32(SPI4_BASE + SPI_CMD) | SPI_CMD_ACT);
	for (uint32_t spin = 0; (rd32(SPI4_BASE + SPI_STATUS0) & SPI_STATUS0_DONE) == 0; spin++) {
		if (spin > 2000000) {
			out("ili: SPI4 transfer NOT DONE (STATUS1 ");
			out_hex(rd32(SPI4_BASE + SPI_STATUS1));
			out(", CMD ");
			out_hex(rd32(SPI4_BASE + SPI_CMD));
			out(", CFG1 ");
			out_hex(rd32(SPI4_BASE + SPI_CFG1));
			out(")\n");
			return 0;
		}
	}
	for (uint32_t w = 0; w < words; w++) {
		uint32_t v = rd32(SPI4_BASE + SPI_RX_DATA);

		for (uint32_t b = 0; b < 4 && rx; b++) {
			if (w * 4 + b < len) {
				rx[w * 4 + b] = (uint8_t)(v >> (8 * b));
			}
		}
	}
	return 1;
}

static void ili_dump(const char *what, const uint8_t *b, uint32_t n)
{
	out(what);
	for (uint32_t i = 0; i < n; i++) {
		out(" ");
		out_hex(b[i]);
	}
	out("\n");
}

/* a register through ICE mode: 0 when either half did not go through */
static int ili_ice_read(uint32_t addr, uint32_t *value)
{
	uint8_t w[5] = { 0x82, 0x25, (uint8_t)addr, (uint8_t)(addr >> 8), (uint8_t)(addr >> 16) };
	uint8_t r[5] = { 0x83, 0, 0, 0, 0 }, in[5] = { 0 };

	if (!spi4_xfer(w, 0, sizeof(w)) || !spi4_xfer(r, in, sizeof(r))) {
		return 0;
	}
	*value = (uint32_t)in[1] | (uint32_t)in[2] << 8 | (uint32_t)in[3] << 16 | (uint32_t)in[4] << 24;
	return 1;
}

/* 1 when the chip answered with its ID (the download needs it) */
static int ili_probe(void)
{
	static const uint32_t pins[] = { ILI_PIN_INT, ILI_PIN_MO, ILI_PIN_MI, ILI_PIN_CLK, ILI_PIN_CS, ILI_PIN_RST, LCM_PIN_RST };
	static const uint8_t ice_on[5] = { 0x82, 0x1F, 0x62, 0x10, 0x18 };
	static const uint8_t ice_off[5] = { 0x82, 0x1B, 0x62, 0x10, 0x18 };
	uint8_t tx[32] = { 0x83 }, rx[32] = { 0 };
	uint32_t pid = 0, pid2 = 0, otp = 0, ana = 0;
	int ok;

	/* a. as LK leaves it: reads only */
	for (uint32_t i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
		out("ili: GPIO");
		out_dec(pins[i]);
		out(" mode ");
		out_dec(gpio_mode_of(pins[i]));
		out(gpio_bit(0x000, pins[i]) ? " out" : " in");
		out(gpio_bit(0x200, pins[i]) ? " high\n" : " low\n");
	}
	out("ili: SPI4 clock gate ");
	out((rd32(INFRACFG_BASE + INFRA2_CG_STA) & INFRA2_SPI4) ? "off" : "on");
	out(", CLK_CFG_2 ");
	out_hex(rd32(TOPCKGEN_BASE + CLK_CFG_2));
	out(" (spi_sel ");
	out_dec((rd32(TOPCKGEN_BASE + CLK_CFG_2) >> 24) & 7);
	out("), EINT 0-31: status ");
	out_hex(rd32(EINT_BASE + 0x000));
	out(" mask ");
	out_hex(rd32(EINT_BASE + 0x080));
	out(" level ");
	out_hex(rd32(EINT_BASE + 0x140));
	out(" polarity ");
	out_hex(rd32(EINT_BASE + 0x300));
	out("\n");
	if (gpio_mode_of(ILI_PIN_CLK) == 1 || gpio_mode_of(ILI_PIN_CS) == 1) {
		out("ili: GPIO144/145 are I2C0 (SCL0/SDA0) as LK leaves them: left alone, nothing more done\n");
		return 0;
	}

	/* b. SPI4 up: its clock gate open (LK's clock kept), its four pins; nothing to the chip */
	wr32(INFRACFG_BASE + INFRA2_CG_CLR, INFRA2_SPI4);
	__asm__ volatile("dsb sy");
	gpio_mode_to(ILI_PIN_MO, ILI_SPI_FUNC);
	gpio_mode_to(ILI_PIN_MI, ILI_SPI_FUNC);
	gpio_mode_to(ILI_PIN_CLK, ILI_SPI_FUNC);
	gpio_mode_to(ILI_PIN_CS, ILI_SPI_FUNC);
	out("ili: SPI4 up: gate ");
	out((rd32(INFRACFG_BASE + INFRA2_CG_STA) & INFRA2_SPI4) ? "STILL OFF" : "on");
	out(", spi_sel ");
	out_dec((rd32(TOPCKGEN_BASE + CLK_CFG_2) >> 24) & 7);
	out(", 6.1 MHz, mode 0; pins 13/14/144/145 at mode ");
	out_dec(gpio_mode_of(ILI_PIN_MO));
	out_dec(gpio_mode_of(ILI_PIN_MI));
	out_dec(gpio_mode_of(ILI_PIN_CLK));
	out_dec(gpio_mode_of(ILI_PIN_CS));
	out("\n");

#if ILI_RELEASE_RST
	/* the chip out of reset: GPIO121 high (DOUT's set register for pins 96..127), then 10 ms */
	wr32(GPIO_BASE + 0x134, 1u << (ILI_PIN_RST % 32));
	__asm__ volatile("dsb sy");
	dbg_delay_ms(10);
	out("ili: reset released: GPIO121 ");
	out(gpio_bit(0x200, ILI_PIN_RST) ? "high\n" : "STILL LOW\n");
#endif

	/* c. one read, nothing written first: the firmware's ACK, if any runs */
	out("ili: c. plain read start\n");
	ok = spi4_xfer(tx, rx, sizeof(rx));
	ili_dump(ok ? "ili: c. plain read:" : "ili: c. plain read FAILED:", rx, sizeof(rx));
	out(rx[1] == 0xa3 ? "ili: c. ACK 0xA3: firmware running\n" : "ili: c. no ACK: no firmware running\n");

	/* d. the ID through ICE mode, the MCU left running */
	out("ili: d. ICE mode (MCU running) start\n");
	if (!spi4_xfer(ice_on, 0, sizeof(ice_on))) {
		out("ili: d. ICE enter not sent\n");
		return 0;
	}
	dbg_delay_ms(1);
	ok = ili_ice_read(0x4009c, &pid) & ili_ice_read(0x400a0, &otp) & ili_ice_read(0x400a4, &ana) &
	    ili_ice_read(0x4009c, &pid2);
	spi4_xfer(ice_off, 0, sizeof(ice_off));
	out("ili: d. ICE mode left\n");
	out(ok ? "ili: d. PID " : "ili: d. a read FAILED; PID ");
	out_hex(pid);
	out(" (again ");
	out_hex(pid2);
	out("), OTP ");
	out_hex(otp);
	out(", ANA ");
	out_hex(ana);
	out((pid >> 16) == 0x7807 && pid == pid2 ? ": chip 0x7807, the ILI7807S\n" : ": NOT 0x7807 (or not the same twice)\n");
	return ok && (pid >> 16) == 0x7807 && pid == pid2;
}

#if ILI_DOWNLOAD
/*
 * ILI_DOWNLOAD=1 (with ILI_PROBE and ILI_RELEASE_RST): the firmware into the chip's RAM, as Android's driver does it
 * on every display resume, without its reset pulse: SPI PLL wake (nine 0xA3), ICE mode with the MCU stopped
 * (0x25), the blocks AP, DATA and TUNING written at their addresses, each one's CRC computed by the chip's DMA and
 * read back from its last four bytes and compared with the file's, the code reset (0xAE to 0x40040), ICE mode left,
 * then the firmware version asked for, and five seconds of reports. ilifw.h is made from YOUR phone's firmware file
 * (mtk/tools/mkilifw.py) and never committed.
 *
 * All of it goes out with the four pins driven by hand (ili_bb_*), mode 0, as the Hot 20's firmware does: the
 * controller in Linux's pause mode (chip select held across FIFO-loads) hung after six of them, and after the pins
 * had been GPIOs it hung on the next transfer at all (STATUS1 busy both times), as the Hot 20's SPI2 would not
 * finish twice in a row either (mtkspi.h).
 */
#include "ilifw.h"
#if !ILI_PROBE || !ILI_RELEASE_RST
#error "ILI_DOWNLOAD needs ILI_PROBE=1 and ILI_RELEASE_RST=1 (LK holds the chip in reset)"
#endif

#define ILI_CHUNK            2048                 /* bytes of a block in one SPI write, as the driver */
static uint8_t ili_buf[5 + ILI_CHUNK + 4];

/* the four SPI4 pins as GPIOs, driven by hand: DOUT/DIR have set and clear registers at +4/+8, 32 pins per 0x10 */
static void ili_pin(uint32_t pin, int high)
{
	wr32(GPIO_BASE + (high ? 0x104 : 0x108) + 0x10 * (pin / 32), 1u << (pin % 32));
}

static void ili_bb_begin(void)
{
	ili_pin(ILI_PIN_CS, 1);                         /* levels first, then outputs, then the pins taken from SPI4 */
	ili_pin(ILI_PIN_CLK, 0);
	ili_pin(ILI_PIN_MO, 0);
	wr32(GPIO_BASE + 0x004 + 0x10 * (ILI_PIN_CS / 32), 1u << (ILI_PIN_CS % 32));
	wr32(GPIO_BASE + 0x004 + 0x10 * (ILI_PIN_CLK / 32), 1u << (ILI_PIN_CLK % 32));
	wr32(GPIO_BASE + 0x004 + 0x10 * (ILI_PIN_MO / 32), 1u << (ILI_PIN_MO % 32));
	wr32(GPIO_BASE + 0x008 + 0x10 * (ILI_PIN_MI / 32), 1u << (ILI_PIN_MI % 32));
	gpio_mode_to(ILI_PIN_CS, 0);
	gpio_mode_to(ILI_PIN_CLK, 0);
	gpio_mode_to(ILI_PIN_MO, 0);
	gpio_mode_to(ILI_PIN_MI, 0);
}

static void ili_bb_end(void)
{
	gpio_mode_to(ILI_PIN_MO, ILI_SPI_FUNC);
	gpio_mode_to(ILI_PIN_MI, ILI_SPI_FUNC);
	gpio_mode_to(ILI_PIN_CLK, ILI_SPI_FUNC);
	gpio_mode_to(ILI_PIN_CS, ILI_SPI_FUNC);
}

/*
 * One transfer, chip select held for all of it: mode 0, MSB first, the data set while the clock is low and read just
 * after it rises; each read of DIN also lets the pad settle. tx NULL sends zeros, rx NULL keeps nothing.
 */
static int ili_bb_xfer(const uint8_t *tx, uint8_t *rx, uint32_t len)
{
	ili_pin(ILI_PIN_CS, 0);
	(void)gpio_bit(0x200, ILI_PIN_MI);
	for (uint32_t i = 0; i < len; i++) {
		uint8_t in = 0;

		for (int bit = 7; bit >= 0; bit--) {
			ili_pin(ILI_PIN_MO, tx ? (tx[i] >> bit) & 1 : 0);
			ili_pin(ILI_PIN_CLK, 1);
			in = (uint8_t)(in << 1 | gpio_bit(0x200, ILI_PIN_MI));
			ili_pin(ILI_PIN_CLK, 0);
			(void)gpio_bit(0x200, ILI_PIN_MI);
		}
		if (rx) {
			rx[i] = in;
		}
	}
	ili_pin(ILI_PIN_CS, 1);
	(void)gpio_bit(0x200, ILI_PIN_MI);
	return 1;
}

static int ili_ice_write(uint32_t addr, uint32_t value, uint32_t n)
{
	uint8_t w[9] = { 0x82, 0x25, (uint8_t)addr, (uint8_t)(addr >> 8), (uint8_t)(addr >> 16) };

	for (uint32_t i = 0; i < n; i++) {
		w[5 + i] = (uint8_t)(value >> (8 * i));
	}
	return ili_bb_xfer(w, 0, 5 + n);
}

/* n bytes of chip memory (n <= 4), in the order they come */
static int ili_ice_bytes(uint32_t addr, uint8_t *b, uint32_t n)
{
	uint8_t w[5] = { 0x82, 0x25, (uint8_t)addr, (uint8_t)(addr >> 8), (uint8_t)(addr >> 16) };
	uint8_t r[5] = { 0x83 }, in[5] = { 0 };

	if (!ili_bb_xfer(w, 0, 5) || !ili_bb_xfer(r, in, 1 + n)) {
		return 0;
	}
	for (uint32_t i = 0; i < n; i++) {
		b[i] = in[1 + i];
	}
	return 1;
}

/* the chip's DMA CRC over [start, start + size), as the driver's calc_hw_dma_crc; 0 and *ok = 0 when it times out */
static uint32_t ili_dma_crc(uint32_t start, uint32_t size, int *ok)
{
	uint8_t b[4] = { 0 };

	*ok = ili_ice_write(0x072104, start, 4) & ili_ice_write(0x072108, 0x80000001, 4) &
	    ili_ice_write(0x072114, 0x0002725c, 4) & ili_ice_write(0x072118, 0x80000000, 4) &
	    ili_ice_write(0x07211c, size, 4) & ili_ice_write(0x041016, 0x00, 1) & ili_ice_write(0x041017, 0x03, 1) &
	    ili_ice_write(0x072100, 0x02000000, 4) & ili_ice_write(0x041016, 0x01, 1) & ili_ice_write(0x048006, 0x2, 1) &
	    ili_ice_write(0x072100, 0x01000000, 4);
	for (int i = 0; i < 50 && *ok; i++) {
		dbg_delay_ms(1);
		if (ili_ice_bytes(0x048006, b, 1) && (b[0] & 2)) {
			break;
		}
		if (i == 49) {
			out("ili: DMA CRC busy for 50 ms\n");
			*ok = 0;
		}
	}
	if (!*ok || !ili_ice_bytes(0x04101c, b, 4)) {
		*ok = 0;
		return 0;
	}
	return (uint32_t)b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16 | (uint32_t)b[3] << 24;
}

static int ili_program(const struct ili_block *k)
{
	for (uint32_t at = 0; at < k->len; at += ILI_CHUNK) {
		uint32_t n = k->len - at < ILI_CHUNK ? k->len - at : ILI_CHUNK, a = k->mem + at;

		ili_buf[0] = 0x82;
		ili_buf[1] = 0x25;
		ili_buf[2] = (uint8_t)a;
		ili_buf[3] = (uint8_t)(a >> 8);
		ili_buf[4] = (uint8_t)(a >> 16);
		for (uint32_t i = 0; i < n; i++) {
			ili_buf[5 + i] = k->data[at + i];
		}
		while (n % 4) {
			ili_buf[5 + n++] = 0xff;
		}
		ili_bb_xfer(ili_buf, 0, 5 + n);
	}
	return 1;
}

static void ili_finger_line(const uint8_t *p)
{
	for (int f = 0; f < 10; f++) {
		const uint8_t *q = p + 1 + 4 * f;
		uint32_t x = (uint32_t)(q[0] & 0xf0) << 4 | q[1], y = (uint32_t)(q[0] & 0x0f) << 8 | q[2];

		if (q[0] == 0xff && q[1] == 0xff && q[2] == 0xff) {
			continue;
		}
		out(" f");
		out_dec(f);
		out(" ");
		out_dec(x);
		out(",");
		out_dec(y);
		out(" -> ");
		out_dec(x * 1079 / 2047);
		out(",");
		out_dec(y * 2411 / 2047);
		out(" p");
		out_dec(q[3]);
	}
}

static void ili_download(void)
{
	static const uint8_t wake[12] = { 0x82, 0x00, 0x09, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3 };
	static const uint8_t ice_stop[5] = { 0x82, 0x25, 0x62, 0x10, 0x18 };
	static const uint8_t ice_off[5] = { 0x82, 0x1B, 0x62, 0x10, 0x18 };
	static const uint8_t ver_pre[5] = { 0x82, 0x00, 0x02, 0xf6, 0x21 };
	static const uint8_t ver_cmd[4] = { 0x82, 0x00, 0x01, 0x21 };
	static uint8_t rx[48];
	uint8_t tx[48] = { 0x83 };
	int good = 1;
	uint32_t reports = 0, lines = 0, t;

	out("ili: 2. download start (SPI4's pins driven by hand): PLL wake, ICE mode with the MCU stopped\n");
	ili_bb_begin();
	if (!ili_bb_xfer(wake, 0, sizeof(wake)) || !ili_bb_xfer(ice_stop, 0, sizeof(ice_stop))) {
		out("ili: 2. ICE enter not sent; nothing written\n");
		ili_bb_end();
		return;
	}
	dbg_delay_ms(1);
	for (uint32_t i = 0; i < sizeof(ili_blocks) / sizeof(ili_blocks[0]) && good; i++) {
		const struct ili_block *k = &ili_blocks[i];
		uint8_t b[4] = { 0 };
		uint32_t dma, iram;
		int ok;

		out("ili: 2. ");
		out(k->name);
		out(" -> ");
		out_hex(k->mem);
		out(", ");
		out_dec(k->len);
		out(" bytes: ");
		if (!ili_program(k)) {
			out("NOT WRITTEN\n");
			good = 0;
			break;
		}
		dma = ili_dma_crc(k->mem, k->len - 4, &ok);
		ok &= ili_ice_bytes(k->mem + k->len - 4, b, 4);
		iram = (uint32_t)b[0] << 24 | (uint32_t)b[1] << 16 | (uint32_t)b[2] << 8 | b[3];
		out("file ");
		out_hex(k->crc);
		out(" dma ");
		out_hex(dma);
		out(" iram ");
		out_hex(iram);
		good = ok && dma == k->crc && iram == k->crc;
		out(good ? " CORRECT\n" : " WRONG\n");
	}
	if (good) {
		out("ili: 2. code reset\n");
		good = ili_ice_write(0x40040, 0xae, 1);
	}
	ili_bb_xfer(ice_off, 0, sizeof(ice_off));
	out("ili: 2. ICE mode left\n");
	if (!good) {
		out("ili: 2. download FAILED; Android's driver loads it again\n");
		ili_bb_end();
		return;
	}
	dbg_delay_ms(100);

	/* the firmware's version, as the driver asks it: the command, the chip's INT pulse, the answer */
	ili_bb_xfer(ver_pre, 0, sizeof(ver_pre));
	ili_bb_xfer(ver_cmd, 0, sizeof(ver_cmd));
	for (t = 0; t < 100000 && gpio_bit(0x200, ILI_PIN_INT); t++) {
		dbg_delay_us(1);
	}
	out(t < 100000 ? "ili: 2. INT came\n" : "ili: 2. no INT in 100 ms\n");
	ili_bb_xfer(tx, rx, 1 + 2 + 9);
	ili_dump("ili: 2. version answer:", rx, 12);
	if (rx[1] == 0xa3 && rx[3] == 0x21) {
		out("ili: 2. firmware ");
		out_dec(rx[4]);
		out(".");
		out_dec(rx[5]);
		out(".");
		out_dec(rx[6]);
		out(".");
		out_dec(rx[7]);
		out(" running\n");
	} else {
		out("ili: 2. no firmware answer\n");
	}

	/* five seconds of reports: each one read when the INT line falls */
	out("ili: 2. touch the screen now: 5 s of reports\n");
	for (t = 0; t < 5000000; t++) {
		if (gpio_bit(0x200, ILI_PIN_INT)) {
			dbg_delay_us(1);
			continue;
		}
		for (uint32_t i = 0; i < sizeof(tx); i++) {
			tx[i] = 0;
		}
		tx[0] = 0x83;
		ili_bb_xfer(tx, rx, 1 + 2 + 43);
		reports++;
		if (lines < 60 && rx[1] == 0xa3 && rx[3] == 0x5a) {
			out("ili: report");
			ili_finger_line(rx + 3);
			out("\n");
			lines++;
		} else if (lines < 60 && rx[1] != 0xa3) {
			ili_dump("ili: report without ACK:", rx, 8);
			lines++;
		}
		for (uint32_t w = 0; w < 2000 && !gpio_bit(0x200, ILI_PIN_INT); w++) {
			dbg_delay_us(1);
		}
	}
	ili_bb_end();
	out("ili: 2. ");
	out_dec(reports);
	out(" reports in 5 s\n");
}
#endif

#endif /* MTKILI_H */
