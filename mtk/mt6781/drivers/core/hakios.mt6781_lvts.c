/*
 * The MT6781's CPU temperature: boot-arg lvts_watch=N says every N s the nine LVTS sensors as MediaTek's
 * thermal driver reads them (three controllers at 0x1100b000, +0x100, +0x200; each result at +0x90 + 4 * n, bits 15..0,
 * bit 16 valid), in °C with the chip's golden value, which mtkloader (DVFS_PROBE=1) passes as lvts_golden=:
 *   lvts: 35 s: A55 41.2 41.0 41.5 cam 40.1 | A76 43.0 43.2 | md 39.0 gpu 39.1 39.0 | max 43.2 | 9 of 9 valid
 * A sensor reading 0 shows "-"; without lvts_golden the raw values are shown. They count only once SSPM runs them
 * (Linux asks it to): readings that never change mean they do not. While their clock (infra_therm: infracfg_ao +0x90
 * bit 10 set) is gated they are not read.
 *
 * Nothing brings LVTS up under XNU (Linux's thermal driver does it, not SSPM), so boot-arg lvts_init=N takes its
 * bring-up in numbered steps, in the order MediaTek's driver has it; writes only to the thermal controllers and the
 * thermal block's reset bit:
 *   1  the block reset (infracfg_ao +0x120/+0x124 bit 0, set then cleared; only that bit written); for each
 *      controller its clock on (+0xe4), its devices reset, their ID read back (+0x4c; expected 0x81 + controller)
 *   2  each controller's devices set up: counting stopped, auto RCK, low-power single mode, its delay, a 20 us
 *      counting window, the chopper and reserved settings (eleven device writes through +0x50, 3 us apart)
 *   3  calibration: each controller's devices to manual RCK; for each sensor, selected, an RC count kicked off and
 *      read back (+0xb0), deselected, and its EDATA (+0x54 + 4 * n) set to RC count * count_r >> 14, count_r from
 *      the chip's efuse (mtkloader's lvts_r=, its nine counts in MediaTek's sensor order)
 *   4  sensing: each controller's sensing off (MONCTL0), its sensor select (+0x40) and calculation scale (+0x48), its
 *      devices in single mode, the polling period and filter (MONCTL1/2: 46.5 us units, 2 of 4 samples, MSRCTL0
 *      +0x38), then its sensing points on (MONCTL0 0x20f, 0x203, 0x207). No interrupt (MONINT) and no hardware
 *      over-temperature trip is set: nothing here can reset the phone.
 */
#include <kern/clock.h>
#include <kern/thread.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.input.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern void IOSleep(unsigned int milliseconds);

#define LVTS_PHYS           0x1100b000ULL
#define INFRA_PHYS          0x10001000ULL
#define INFRA_CG0_STA       0x90
#define INFRA_THERM_CG      (1u << 10)
#define INFRA_RST0_SET      0x120
#define INFRA_RST0_CLR      0x124
#define INFRA_RST0_THERM    (1u << 0)
#define LVTS_ID             0x4c
#define LVTS_CONFIG         0x50                /* a write here is a write to the controller's sensor devices */
#define LVTS_CLKEN          0xe4
#define DEV_WRITE(reg, v)   (0xc1030000u | (reg) << 8 | (v))
#define DEV_READ_ID         0xc502fc55u         /* the devices' ID read back into LVTS_ID */
#define DEV_READ(reg)       (0xc1020000u | (reg) << 8)  /* a device register into LVTS_RDATA */
#define CFG_ACCESS          (1u << 24)          /* LVTS_CONFIG: a device access under way */
#define CFG_SENSING         (1u << 25)          /* LVTS_CONFIG: a device counting */
#define LVTS_EDATA(n)       (0x54 + 4 * (n))
#define LVTS_MONCTL0        0x00                /* the sensing points on, bit per point, and 0x200 */
#define LVTS_MONCTL1        0x04
#define LVTS_MONCTL2        0x08
#define LVTS_MONINT         0x0c
#define LVTS_MSRCTL0        0x38
#define LVTS_TSSEL          0x40
#define LVTS_CALSCALE       0x48
#define LVTS_RDATA          0xb0

static const struct {
	const char *name;
	uint32_t at;
} sensor[9] = {
	{ " A55", 0x090 }, { "", 0x094 }, { "", 0x098 }, { " cam", 0x09c },
	{ " | A76", 0x190 }, { "", 0x194 },
	{ " | md", 0x290 }, { " gpu", 0x294 }, { "", 0x298 },
};

static vm_offset_t lvts, infra;
static uint32_t every_s, golden;

extern void IODelay(unsigned int microseconds);

/* step 2's device writes: register, value */
static const uint8_t dev_setup[][2] = {
	{ 0x03, 0x00 },         /* counting stopped (RG_TSFM_ST = 0) */
	{ 0x0e, 0x01 },         /* auto RCK */
	{ 0x06, 0xb8 },         /* low-power single mode */
	{ 0x07, 0xa6 },         /* RG_TSFM_LPDLY = 2 */
	{ 0x05, 0x00 },         /* counting window: 20 us */
	{ 0x04, 0x20 },
	{ 0x0a, 0xac },         /* TSV2F_CHOP_CKSEL, TSV2F_EN */
	{ 0x0c, 0xfc },         /* TSBG_DEM_CKSEL, TSBG_CHOP_EN */
	{ 0x09, 0x8d },         /* TS_RSV */
	{ 0x08, 0xf7 },         /* TS_CHOP control */
	{ 0x0b, 0x04 },         /* TSV2F_RSV */
};

static void
wr(vm_offset_t at, uint32_t v)
{
	__asm__ volatile ("dsb sy" ::: "memory");
	*(volatile uint32_t *)at = v;
	__asm__ volatile ("dsb sy" ::: "memory");
}

/* step 1: the block reset, then each controller's clock on, its devices reset, their ID read back */
static void
init_step1(void)
{
	printf("lvts init 1: infracfg_ao reset 0 set/clear read %08x/%08x, clock gates %08x\n",
	    *(volatile uint32_t *)(infra + INFRA_RST0_SET), *(volatile uint32_t *)(infra + INFRA_RST0_CLR),
	    *(volatile uint32_t *)(infra + INFRA_CG0_STA));
	wr(infra + INFRA_RST0_SET, INFRA_RST0_THERM);
	IODelay(10);
	wr(infra + INFRA_RST0_CLR, INFRA_RST0_THERM);
	IODelay(10);
	for (uint32_t c = 0; c < 3; c++) {
		vm_offset_t tc = lvts + 0x100 * c;
		uint32_t id;

		wr(tc + LVTS_CLKEN, 1);
		wr(tc + LVTS_CONFIG, DEV_WRITE(0xff, 0xff));
		IODelay(100);
		wr(tc + LVTS_CONFIG, DEV_READ_ID);
		IODelay(3);
		id = *(volatile uint32_t *)(tc + LVTS_ID);
		printf("lvts init 1: controller %u: clock %08x, ID %08x (%s 0x%x)\n", c, *(volatile uint32_t *)(tc + LVTS_CLKEN),
		    id, (id & 0xff) == 0x81 + c ? "as expected:" : "NOT the expected", 0x81 + c);
	}
}

/* " <name> <°C, one decimal>" at line + at */
static int
put_c(char *line, int at, int size, const char *name, int mc)
{
	int d = (mc < 0 ? -mc : mc) / 100;

	return snprintf(line + at, size - at, "%s %s%d.%d", name, mc < 0 ? "-" : "", d / 10, d % 10);
}

/*
 * For MT6781_DVFS: how many of the nine sensors have a reading now, and the hottest of those in m°C (*mc). A reading is
 * a non-zero raw value, as MediaTek's driver takes it: the valid bit (16) is only reported there, and reading a sensor
 * seems to clear it (two reads in a row: 9 valid, then 0).
 */
int
hakios_mt6781_lvts_hottest(int *mc)
{
	int valid = 0;

	if (lvts == 0 || golden == 0 || (*(volatile uint32_t *)(infra + INFRA_CG0_STA) & INFRA_THERM_CG)) {
		return 0;
	}
	for (int i = 0; i < 9; i++) {
		uint32_t raw = *(volatile uint32_t *)(lvts + sensor[i].at);
		int t = (int)((-204650LL * (raw & 0xffff)) >> 14) + 500 * (int)golden + 204650;

		if (raw & 0xffff) {
			*mc = valid++ ? MAX(*mc, t) : t;
		}
	}
	return valid;
}

static void
watch(__unused void *arg, __unused wait_result_t wr)
{
	for (;;) {
		char line[256];
		int at, max = 0, any = 0, valid = 0;
		uint64_t ns;

		absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
		at = snprintf(line, sizeof(line), "lvts: %llu s:", ns / NSEC_PER_SEC);
		if (*(volatile uint32_t *)(infra + INFRA_CG0_STA) & INFRA_THERM_CG) {
			printf("%s the thermal controller's clock is gated: not read\n", line);
			IOSleep(every_s * 1000);
			continue;
		}
		for (int i = 0; i < 9 && at < (int)sizeof(line); i++) {
			uint32_t raw = *(volatile uint32_t *)(lvts + sensor[i].at);
			int mc = (int)((-204650LL * (raw & 0xffff)) >> 14) + 500 * (int)golden + 204650;

			valid += (raw >> 16) & 1;
			if (golden == 0) {
				at += snprintf(line + at, sizeof(line) - at, "%s %05x", sensor[i].name, raw);
			} else if ((raw & 0xffff) == 0) {
				at += snprintf(line + at, sizeof(line) - at, "%s -", sensor[i].name);
			} else {
				at += put_c(line, at, sizeof(line), sensor[i].name, mc);
				max = any ? MAX(max, mc) : mc;
				any = 1;
			}
		}
		if (any && at < (int)sizeof(line)) {
			at += put_c(line, at, sizeof(line), " | max", max);
		}
		printf("%s | %d of 9 valid\n", line, valid);
		IOSleep(every_s * 1000);
	}
}

/* step 2: each controller's devices set up, in the order MediaTek's driver writes them */
static void
init_step2(void)
{
	for (uint32_t c = 0; c < 3; c++) {
		vm_offset_t tc = lvts + 0x100 * c;

		for (uint32_t i = 0; i < sizeof(dev_setup) / sizeof(dev_setup[0]); i++) {
			wr(tc + LVTS_CONFIG, DEV_WRITE(dev_setup[i][0], dev_setup[i][1]));
			IODelay(3);
		}
		printf("lvts init 2: controller %u: %zu device writes; LVTS_CONFIG %08x, ID %08x\n", c,
		    sizeof(dev_setup) / sizeof(dev_setup[0]), *(volatile uint32_t *)(tc + LVTS_CONFIG),
		    *(volatile uint32_t *)(tc + LVTS_ID));
	}
}

static void
dev_wr(vm_offset_t tc, uint32_t reg, uint32_t v)
{
	wr(tc + LVTS_CONFIG, DEV_WRITE(reg, v));
	IODelay(3);
}

/* whether the bit of LVTS_CONFIG went to 0 within 200 us */
static bool
cfg_clear(vm_offset_t tc, uint32_t bit)
{
	for (int i = 0; i < 100; i++) {
		if (!(*(volatile uint32_t *)(tc + LVTS_CONFIG) & bit)) {
			return true;
		}
		IODelay(2);
	}
	return false;
}

/* step 3: the RC calibration, and each sensor's EDATA from it and the efuse's count_r */
static void
init_step3(void)
{
	static const uint32_t sensors[3] = { 4, 2, 3 };
	char arg[96] = "";
	uint32_t count_r[9], n = 0, s = 0;

	if (PE_parse_boot_argn("lvts_r", arg, sizeof(arg)) && arg[0] == 'r') {
		for (const char *p = arg + 1; n < 9 && *p;) {
			uint32_t v = 0;

			for (; (*p >= '0' && *p <= '9') || (*p >= 'a' && *p <= 'f'); p++) {
				v = v << 4 | (uint32_t)(*p <= '9' ? *p - '0' : *p - 'a' + 10);
			}
			count_r[n++] = v;
			if (*p != '.') {
				break;
			}
			p++;
		}
	}
	for (uint32_t i = 0; i < n; i++) {
		if (count_r[i] == 0 || count_r[i] > 0xffffff) {
			n = 0;
		}
	}
	if (n != 9) {
		printf("lvts init 3: no lvts_r= with nine counts from mtkloader (\"%s\"): no calibration\n", arg);
		return;
	}
	for (uint32_t c = 0; c < 3; c++) {
		vm_offset_t tc = lvts + 0x100 * c;

		dev_wr(tc, 0x0e, 0x00);                 /* manual RCK */
		for (uint32_t j = 0; j < sensors[c]; j++, s++) {
			uint32_t rc, edata;
			bool counted, read;

			dev_wr(tc, 0x0d, j);                /* sensor j, with RCK */
			dev_wr(tc, 0x06, 0xb8);             /* low-power single mode */
			dev_wr(tc, 0x05, 0x00);             /* counting window: 20 us */
			dev_wr(tc, 0x04, 0x20);
			dev_wr(tc, 0x03, 0x02);             /* RC counting kicked off */
			IODelay(50);
			counted = cfg_clear(tc, CFG_SENSING);
			wr(tc + LVTS_CONFIG, DEV_READ(0x00));
			IODelay(5);
			read = cfg_clear(tc, CFG_ACCESS);
			rc = *(volatile uint32_t *)(tc + LVTS_RDATA) & 0xffffff;
			dev_wr(tc, 0x0d, 0x10 | j);         /* sensor j, without RCK: as for reading temperatures */
			edata = (uint32_t)(((uint64_t)rc * count_r[s]) >> 14);
			wr(tc + LVTS_EDATA(j), edata);
			printf("lvts init 3: sensor %u (controller %u, %u): count_r %x, RC now %x%s%s, EDATA %x (reads %x)\n", s, c,
			    j, count_r[s], rc, counted ? "" : " (still counting after 200 us)", read ? "" : " (read not done)",
			    edata, *(volatile uint32_t *)(tc + LVTS_EDATA(j)));
		}
	}
}

/* step 4: the controllers set to measure, then their sensing points on */
static void
init_step4(void)
{
	static const uint32_t points[3] = { 0x20f, 0x203, 0x207 };      /* four, two, three sensors */

	for (uint32_t c = 0; c < 3; c++) {
		wr(lvts + 0x100 * c + LVTS_MONCTL0, 0);
	}
	for (uint32_t c = 0; c < 3; c++) {
		vm_offset_t tc = lvts + 0x100 * c;

		wr(tc + LVTS_TSSEL, 0x13121110);
		wr(tc + LVTS_CALSCALE, 0x300);
		dev_wr(tc, 0x06, 0xb8);                 /* single mode */
		wr(tc + LVTS_MONCTL1, 1u << 20 | 0x00c); /* group delay 1, period 12 x 256 bus clocks: 46.5 us */
		wr(tc + LVTS_MONCTL2, 1u << 16 | 0x001); /* filter and sensor interval 1 period */
		wr(tc + LVTS_MSRCTL0, 0x492);           /* 2 of 4 samples */
		IODelay(1);
	}
	for (uint32_t c = 0; c < 3; c++) {
		vm_offset_t tc = lvts + 0x100 * c;

		wr(tc + LVTS_MONCTL0, points[c]);
		printf("lvts init 4: controller %u: MONCTL0 %08x, MONCTL1 %08x, MONCTL2 %08x, MSRCTL0 %08x, MONINT %08x\n", c,
		    *(volatile uint32_t *)(tc + LVTS_MONCTL0), *(volatile uint32_t *)(tc + LVTS_MONCTL1),
		    *(volatile uint32_t *)(tc + LVTS_MONCTL2), *(volatile uint32_t *)(tc + LVTS_MSRCTL0),
		    *(volatile uint32_t *)(tc + LVTS_MONINT));
	}
}

void
hakios_mt6781_lvts_init(void)
{
	thread_t th;
	uint32_t step = 0;

	PE_parse_boot_argn("lvts_init", &step, sizeof(step));
	PE_parse_boot_argn("lvts_watch", &every_s, sizeof(every_s));
	if (step == 0 && every_s == 0) {
		return;
	}
	PE_parse_boot_argn("lvts_golden", &golden, sizeof(golden));
	lvts = ml_io_map(LVTS_PHYS, 0x1000);
	infra = ml_io_map(INFRA_PHYS, 0x1000);
	if (step >= 1) {
		if (*(volatile uint32_t *)(infra + INFRA_CG0_STA) & INFRA_THERM_CG) {
			printf("lvts init: the thermal controller's clock is gated: no bring-up\n");
		} else {
			init_step1();
			if (step >= 2) {
				init_step2();
			}
			if (step >= 3) {
				init_step3();
			}
			if (step >= 4) {
				init_step4();
			}
		}
	}
	if (every_s == 0) {
		return;
	}
	if (kernel_thread_start(watch, NULL, &th) != KERN_SUCCESS) {
		printf("lvts: no thread; the temperature is not watched\n");
		return;
	}
	thread_deallocate(th);
	printf("lvts: the CPU temperature every %u s (lvts_watch=), golden %u%s\n", every_s, golden,
	    golden ? "" : ": none given, raw values");
}
