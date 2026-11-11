/*
 * SSPM's CPU DVFS on the MT6781, started as MediaTek's hybrid DVFS driver starts it (cpuhvfs, SSPM v2): boot-arg
 * dvfs_step=N, steps 1 to N, from a kernel thread, and nothing written before all nine LVTS sensors have a reading
 * (MT6781_LVTS, lvts_init=4):
 *   1  read only: LK's tables (mtkloader's dvfs_tbl=) against the CPU DVFS SRAM ("CSRAM") Android left: all 48 rows
 *      the same MHz and dividers, the guards there, the CCI map and IMAX table there (Android's: none after a cold
 *      power-on, and then nothing is done); the PLLs and the current OPP, Vproc and request words said
 *   2  the CSRAM as Linux lays it out: the CCI map and IMAX table kept aside, the guards, everything from +0x2a0 to the
 *      end cleared, the PPM limits (A55s all OPPs; A76s capped at OPP 7), the table from LK's (voltage step << 16 |
 *      MHz, clock divider << 16 | post divider), the CCI map, IMAX table and IMAX_EN back, IMAX_THERMAL_EN 1; the
 *      requests where LK left the clocks (A55s OPP 2, A76s OPP 11)
 *   3  SSPM told, on its CPU DVFS channel (mailbox 0 slot 26, the answer in mailbox 2 slot 24): INIT_PTBL (segment
 *      0), INIT (level 0), the scheduler's assist off (+0x2bc), CLUSTER_ON_OFF for both clusters; GET_FREQ of the A76s
 *   4  the A76s asked for OPP 7 (1530 MHz, request word +0x380); the PLL, GET_FREQ and the temperature after 100 ms
 * Then every 500 ms: at 75 °C or more on any sensor, the A76s asked for OPP 11 again, for the rest of the run; every
 * 5 s the A76s' PLL, the current OPPs and the hottest sensor said; what SSPM sends on its CPU DVFS notice channel
 * (mailbox 2 slot 10) read, said and cleared. A message not taken in 100 ms, or not answered in time: no further
 * step, the A76s back to OPP 11. mtkloader built with DVFS_RESET_END=1 ends such a run in a watchdog reset. Every line
 * also goes onto darwin at once with mtkufs_crumbs=1 (MT6781_UFS breadcrumbs), and one before each write that could
 * reset the phone.
 */
#include <kern/clock.h>
#include <stdarg.h>
#include <kern/thread.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.input.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern void IOSleep(unsigned int milliseconds);
extern void IODelay(unsigned int microseconds);

#define CSRAM_PHYS          0x0011bc00ULL
#define CSRAM_SIZE          0x1400
#define MBOX_PHYS(n)        (0x10450000ULL + 0x10000ULL * (n))
#define MBOX_SET            0x1000
#define MBOX_CLR            0x1004
#define APMIXED_PHYS        0x1000c000ULL
#define ARMPLL_LL_CON1      0x204
#define ARMPLL_L_CON1       0x214

#define GUARD0              0x55aa55aau
#define GUARD1              0xaa55aa55u
#define OFFS_TBL_S          0x010               /* the A55s' table; the A76s' at +0x90 from it, the CCI's at +0x120 */
#define OFFS_TBL_E          0x250
#define OFFS_DATA_S         0x2a0
#define OFFS_SCHED_DIS      0x2bc
#define OFFS_PPM_LIMIT      0x320               /* per cluster: highest allowed OPP << 16 | lowest */
#define OFFS_CUR_VPROC      0x32c
#define OFFS_CUR_FREQ       0x354
#define OFFS_WFI            0x37c               /* per cluster: the OPP asked for */
#define OFFS_CCI_TBL        0xfa0
#define OFFS_IMAX_TBL       0x11a0
#define OFFS_IMAX_EN        0x11c4
#define OFFS_IMAX_THERMAL   0x11d4
#define CCI_WORDS           ((OFFS_IMAX_TBL - OFFS_CCI_TBL) / 4)
#define IMAX_WORDS          ((OFFS_IMAX_EN - OFFS_IMAX_TBL) / 4)

#define SEND_SLOT           26                  /* IPIS_C_GPU_DVFS: mailbox 0, 4 words, its bit 26 */
#define REPLY_SLOT          24                  /* its answer: mailbox 2, 1 word, bit 24 */
#define NOTICE_SLOT         10                  /* IPIR_C_GPU_DVFS: SSPM's notices, mailbox 2, 4 words, bit 10 */
#define IPI_INIT_PTBL       0
#define IPI_INIT            1
#define IPI_CLUSTER_ON_OFF  2
#define IPI_GET_FREQ        6

#define A55                 0
#define A76                 1
#define A55_OPP_LK          2                   /* 1866 MHz: LK leaves its PLL at 1865 */
#define A76_OPP_LK          11                  /* 1085 MHz: LK leaves its PLL at 1084 */
#define A76_OPP_UP          7                   /* 1530 MHz */
#define A76_CAP             7                   /* the A76s' PPM limit: OPP 7 and slower only */
#define HOT_MC              75000

static vm_offset_t csram, mbox0, mbox2, apmixed;
static const uint32_t *tbl;                     /* mtkloader's copy of LK's three tables: 3 x 16 rows of 4 */
static uint32_t step, notices;

/* a line said, and with mtkufs_crumbs=1 also put onto darwin at once (MT6781_UFS): it outlasts a reset */
static void say(const char *format, ...) __printflike(1, 2);

static void
say(const char *format, ...)
{
	char line[480];
	va_list ap;

	va_start(ap, format);
	vsnprintf(line, sizeof(line), format, ap);
	va_end(ap);
	printf("%s\n", line);
	(void)hakios_mt6781_ufs_crumb(line);
}

static uint32_t
rd(vm_offset_t at)
{
	return *(volatile uint32_t *)at;
}

static void
wr(vm_offset_t at, uint32_t v)
{
	__asm__ volatile ("dsb sy" ::: "memory");
	*(volatile uint32_t *)at = v;
	__asm__ volatile ("dsb sy" ::: "memory");
}

static uint32_t
pll_mhz(uint32_t con1_at)
{
	uint32_t con1 = rd(apmixed + con1_at), post = (con1 >> 24) & 7;

	return (uint32_t)((((uint64_t)(con1 & 0x3fffff) * 26) >> 14) >> (post < 4 ? post : 4));
}

/* SSPM's notices on the CPU DVFS channel: read, said, cleared */
static void
take_notices(void)
{
	if (rd(mbox2 + MBOX_CLR) & (1u << NOTICE_SLOT)) {
		uint32_t w[4];

		for (int i = 0; i < 4; i++) {
			w[i] = rd(mbox2 + 4 * (NOTICE_SLOT + i));
		}
		wr(mbox2 + MBOX_CLR, 1u << NOTICE_SLOT);
		if (notices++ < 20) {
			say("dvfs: SSPM's notice %u: %08x %08x %08x %08x", notices, w[0], w[1], w[2], w[3]);
		}
	}
}

/* one command on the CPU DVFS channel and its answer, polled; 0, or what went wrong said and an error */
static int
ipi(const char *what, uint32_t cmd, uint32_t a0, uint32_t a1, uint32_t wait_ms, uint32_t *reply)
{
	uint32_t us;

	say("dvfs: %s: sending now", what);
	if ((rd(mbox0 + MBOX_SET) & (1u << SEND_SLOT)) || (rd(mbox2 + MBOX_CLR) & (1u << REPLY_SLOT))) {
		say("dvfs: %s: the channel is not clear (mailbox 0 set %08x, mailbox 2 clear %08x): not sent", what,
		    rd(mbox0 + MBOX_SET), rd(mbox2 + MBOX_CLR));
		return EBUSY;
	}
	wr(mbox0 + 4 * SEND_SLOT, cmd);
	wr(mbox0 + 4 * (SEND_SLOT + 1), a0);
	wr(mbox0 + 4 * (SEND_SLOT + 2), a1);
	wr(mbox0 + 4 * (SEND_SLOT + 3), 0);
	wr(mbox0 + MBOX_SET, 1u << SEND_SLOT);
	for (us = 0; (rd(mbox0 + MBOX_SET) & (1u << SEND_SLOT)) && us < 100000; us += 10) {
		IODelay(10);
	}
	if (rd(mbox0 + MBOX_SET) & (1u << SEND_SLOT)) {
		say("dvfs: %s: not taken in 100 ms", what);
		return ETIMEDOUT;
	}
	for (us = 0; !(rd(mbox2 + MBOX_CLR) & (1u << REPLY_SLOT)) && us < wait_ms * 1000; us += 10) {
		IODelay(10);
	}
	if (!(rd(mbox2 + MBOX_CLR) & (1u << REPLY_SLOT))) {
		say("dvfs: %s: taken, no answer in %u ms", what, wait_ms);
		return ETIMEDOUT;
	}
	*reply = rd(mbox2 + 4 * REPLY_SLOT);
	wr(mbox2 + MBOX_CLR, 1u << REPLY_SLOT);
	say("dvfs: %s: answered after %u us: %u (%08x)", what, us, *reply, *reply);
	return 0;
}

static void
say_state(const char *when)
{
	int mc = 0, valid = hakios_mt6781_lvts_hottest(&mc);

	say("dvfs (%s): PLL A55s %u MHz, A76s %u MHz; OPP now %u/%u/%u, Vproc %x/%x/%x, asked %u/%u, limits %08x/%08x; "
	    "hottest %d.%d C (%d read); %u notices", when, pll_mhz(ARMPLL_LL_CON1), pll_mhz(ARMPLL_L_CON1),
	    rd(csram + OFFS_CUR_FREQ), rd(csram + OFFS_CUR_FREQ + 4), rd(csram + OFFS_CUR_FREQ + 8),
	    rd(csram + OFFS_CUR_VPROC), rd(csram + OFFS_CUR_VPROC + 4), rd(csram + OFFS_CUR_VPROC + 8),
	    rd(csram + OFFS_WFI), rd(csram + OFFS_WFI + 4), rd(csram + OFFS_PPM_LIMIT), rd(csram + OFFS_PPM_LIMIT + 4),
	    mc / 1000, (mc < 0 ? -mc : mc) % 1000 / 100, valid, notices);
}

/* step 1: whether LK's tables and Android's CSRAM agree, and what step 2 needs is there */
static bool
check(void)
{
	static const uint32_t at[3] = { OFFS_TBL_S, OFFS_TBL_S + 0x90, OFFS_TBL_S + 0x120 };
	uint32_t same = 0, cci = 0, imax = 0;

	for (uint32_t t = 0; t < 3; t++) {
		for (uint32_t r = 0; r < 16; r++) {
			const uint32_t *row = tbl + 64 * t + 4 * r;
			uint32_t f = rd(csram + at[t] + 4 * r), d = rd(csram + at[t] + 4 * (16 + r));

			same += (f & 0xffff) == row[0] && d == ((row[3] & 0xff) << 16 | (row[2] & 0xff));
		}
	}
	for (uint32_t i = 0; i < CCI_WORDS; i++) {
		cci += rd(csram + OFFS_CCI_TBL + 4 * i) != 0;
	}
	for (uint32_t i = 0; i < IMAX_WORDS; i++) {
		imax += rd(csram + OFFS_IMAX_TBL + 4 * i) != 0;
	}
	say("dvfs 1: LK's tables and the CSRAM: %u of 48 rows agree; guards %08x %08x %08x %08x, %08x at +0x2a0; CCI map "
	    "%u of %u words set, IMAX %u of %u, IMAX_EN %u", same, rd(csram), rd(csram + 4), rd(csram + 8), rd(csram + 12),
	    rd(csram + OFFS_DATA_S), cci, CCI_WORDS, imax, IMAX_WORDS, rd(csram + OFFS_IMAX_EN));
	say_state("as found");
	if (same != 48 || rd(csram) != GUARD0 || rd(csram + 4) != GUARD1 || rd(csram + 8) != GUARD0 ||
	    rd(csram + 12) != GUARD1 || cci == 0 || imax == 0) {
		say("dvfs 1: not as Android leaves it (after a cold power-on there is no CCI map or IMAX table): no DVFS "
		    "writes");
		return false;
	}
	return true;
}

/* step 2: the CSRAM laid out as Linux's init_cpuhvfs_debug_repo and cpuhvfs_pvt_tbl_create leave it */
static void
lay_out(void)
{
	static uint32_t cci[CCI_WORDS], imax[IMAX_WORDS];
	uint32_t imax_en = rd(csram + OFFS_IMAX_EN);

	say("dvfs 2: laying out the CSRAM now");

	for (uint32_t i = 0; i < CCI_WORDS; i++) {
		cci[i] = rd(csram + OFFS_CCI_TBL + 4 * i);
	}
	for (uint32_t i = 0; i < IMAX_WORDS; i++) {
		imax[i] = rd(csram + OFFS_IMAX_TBL + 4 * i);
	}
	wr(csram, GUARD0);
	wr(csram + 4, GUARD1);
	wr(csram + 8, GUARD0);
	wr(csram + 12, GUARD1);
	for (uint32_t o = OFFS_DATA_S; o < CSRAM_SIZE; o += 4) {
		wr(csram + o, 0);
	}
	wr(csram + OFFS_DATA_S, GUARD0);
	wr(csram + OFFS_PPM_LIMIT + 4 * A55, 0u << 16 | 15);
	wr(csram + OFFS_PPM_LIMIT + 4 * A76, (uint32_t)A76_CAP << 16 | 15);
	for (uint32_t o = OFFS_TBL_S; o < OFFS_TBL_E; o += 4) {
		wr(csram + o, 0);
	}
	for (uint32_t t = 0; t < 3; t++) {
		vm_offset_t base = csram + OFFS_TBL_S + 4 * 36 * t;

		for (uint32_t r = 0; r < 16; r++) {
			const uint32_t *row = tbl + 64 * t + 4 * r;

			wr(base + 4 * r, (row[1] & 0xfff) << 16 | (row[0] & 0xffff));
			wr(base + 4 * (16 + r), (row[3] & 0xff) << 16 | (row[2] & 0xff));
		}
	}
	for (uint32_t t = 0; t < 4; t++) {
		wr(csram + OFFS_TBL_S + 4 * (32 + 36 * t), 0xffffffff);   /* each table's end, as Linux marks it */
	}
	for (uint32_t i = 0; i < CCI_WORDS; i++) {
		wr(csram + OFFS_CCI_TBL + 4 * i, cci[i]);
	}
	for (uint32_t i = 0; i < IMAX_WORDS; i++) {
		wr(csram + OFFS_IMAX_TBL + 4 * i, imax[i]);
	}
	wr(csram + OFFS_IMAX_EN, imax_en);
	wr(csram + OFFS_IMAX_THERMAL, 1);
	wr(csram + OFFS_WFI + 4 * A55, A55_OPP_LK);
	wr(csram + OFFS_WFI + 4 * A76, A76_OPP_LK);
	say("dvfs 2: the CSRAM laid out: table from LK's, CCI map and IMAX kept, IMAX_EN %u, limits %08x/%08x, asked "
	    "%u/%u", rd(csram + OFFS_IMAX_EN), rd(csram + OFFS_PPM_LIMIT), rd(csram + OFFS_PPM_LIMIT + 4),
	    rd(csram + OFFS_WFI), rd(csram + OFFS_WFI + 4));
}

static void
a76_back(const char *why)
{
	wr(csram + OFFS_WFI + 4 * A76, A76_OPP_LK);
	say("dvfs: %s: the A76s asked for OPP %u again", why, A76_OPP_LK);
}

static void
run(__unused void *arg, __unused wait_result_t wr_)
{
	uint32_t reply = 0, waited = 0;
	int mc = 0;
	bool cooled = false;

	if (!check() || step < 2) {
		return;
	}
	while (hakios_mt6781_lvts_hottest(&mc) != 9) {
		if (waited++ == 120) {
			say("dvfs: the temperature sensors not all read after 120 s (lvts_init=4 lvts_watch=): no DVFS "
			    "writes");
			return;
		}
		IOSleep(1000);
	}
	say("dvfs: all nine sensors read, hottest %d.%d C: starting", mc / 1000, mc % 1000 / 100);
	lay_out();
	if (step < 3) {
		return;
	}
	if (ipi("INIT_PTBL (segment 0)", IPI_INIT_PTBL, 0, 0, 2000, &reply) ||
	    ipi("INIT (level 0)", IPI_INIT, 0, 0, 2000, &reply)) {
		a76_back("start not answered");
		return;
	}
	say("dvfs 3: the scheduler's assist off (+0x2bc) now");
	wr(csram + OFFS_SCHED_DIS, 1);
	if (ipi("CLUSTER_ON_OFF (A55s on)", IPI_CLUSTER_ON_OFF, A55, 1, 10000, &reply) ||
	    ipi("CLUSTER_ON_OFF (A76s on)", IPI_CLUSTER_ON_OFF, A76, 1, 10000, &reply)) {
		a76_back("cluster on not answered");
		return;
	}
	IOSleep(100);
	(void)ipi("GET_FREQ (A76s' PLL)", IPI_GET_FREQ, A76, 0, 2000, &reply);
	say_state("started");
	if (step >= 4) {
		uint32_t mhz;

		say("dvfs 4: the A76s asked for OPP %u now", A76_OPP_UP);
		wr(csram + OFFS_WFI + 4 * A76, A76_OPP_UP);
		IOSleep(100);
		mhz = pll_mhz(ARMPLL_L_CON1);
		(void)ipi("GET_FREQ (A76s' PLL, OPP 7 asked)", IPI_GET_FREQ, A76, 0, 2000, &reply);
		say_state("A76s at OPP 7");
		if (mhz < 1450 || mhz > 1610) {
			a76_back("the A76s' PLL is not at 1530 MHz");
			cooled = true;
		}
	}
	for (uint32_t tick = 1;; tick++) {
		IOSleep(500);
		take_notices();
		if (!cooled && hakios_mt6781_lvts_hottest(&mc) > 0 && mc >= HOT_MC) {
			a76_back("75 C reached");
			cooled = true;
		}
		if (tick % 10 == 0) {
			say_state("running");
		}
	}
}

void
hakios_mt6781_dvfs_init(void)
{
	uint32_t at = 0;
	thread_t th;

	if (!PE_parse_boot_argn("dvfs_step", &step, sizeof(step)) || step == 0) {
		return;
	}
	if (!PE_parse_boot_argn("dvfs_tbl", &at, sizeof(at)) || at == 0) {
		say("dvfs: no dvfs_tbl= from mtkloader (DVFS_PROBE=1): nothing done");
		return;
	}
	tbl = (const uint32_t *)(ml_io_map(at & ~0xfffULL, 0x2000) + (at & 0xfff));
	csram = ml_io_map(CSRAM_PHYS & ~0xfffULL, 0x2000) + (CSRAM_PHYS & 0xfff);
	mbox0 = ml_io_map(MBOX_PHYS(0), 0x2000);
	mbox2 = ml_io_map(MBOX_PHYS(2), 0x2000);
	apmixed = ml_io_map(APMIXED_PHYS, 0x1000);
	if (kernel_thread_start(run, NULL, &th) != KERN_SUCCESS) {
		say("dvfs: no thread: nothing done");
		return;
	}
	thread_deallocate(th);
	say("dvfs: SSPM's CPU DVFS, steps 1 to %u (dvfs_step=), LK's tables at %x", step, at);
}
