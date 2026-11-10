/*
 * The MT6781's watchdog (TOPRGU at 0x10007000), kept from firing during a run: boot-arg mtk_wdt_kick=1. mtkloader
 * arms it before XNU (31 s, the most its length takes); a kernel thread here restarts its count every 5 s by writing
 * the restart key to WDT_RST, as MediaTek's mtk_wdt driver pings it. Nothing else of the watchdog is written: mode
 * and length stay as mtkloader set them. So a run can last longer than 31 s (loader_return_after= still ends it), and
 * a kernel that stops scheduling threads -- a hang with interrupts off, a panic spinning -- still ends in its reset.
 */
#include <kern/thread.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.input.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern void IOSleep(unsigned int milliseconds);

#define TOPRGU_PHYS         0x10007000ULL
#define WDT_MODE            0x00
#define WDT_MODE_EN         (1u << 0)
#define WDT_LENGTH          0x04
#define WDT_RST             0x08
#define WDT_RST_RELOAD      0x1971
#define KICK_MS             5000

static vm_offset_t rgu;
static uint64_t kicks;

static void
kick(void)
{
	__asm__ volatile ("dsb sy" ::: "memory");
	*(volatile uint32_t *)(rgu + WDT_RST) = WDT_RST_RELOAD;
	__asm__ volatile ("dsb sy" ::: "memory");
	kicks++;
}

static void
kicker(__unused void *arg, __unused wait_result_t wr)
{
	for (;;) {
		kick();
		IOSleep(KICK_MS);
	}
}

void
hakios_mt6781_wdt_init(void)
{
	uint32_t on = 0, mode;
	thread_t th;

	if (!PE_parse_boot_argn("mtk_wdt_kick", &on, sizeof(on)) || on == 0) {
		return;
	}
	rgu = ml_io_map(TOPRGU_PHYS, 0x1000);
	mode = *(volatile uint32_t *)(rgu + WDT_MODE);
	printf("mt6781wdt: TOPRGU mode %08x, length %08x as mtkloader left it\n", mode, *(volatile uint32_t *)(rgu + WDT_LENGTH));
	if ((mode & WDT_MODE_EN) == 0) {
		printf("mt6781wdt: the watchdog is off: nothing to kick\n");
		return;
	}
	if (kernel_thread_start(kicker, NULL, &th) != KERN_SUCCESS) {
		printf("mt6781wdt: no thread; the watchdog is NOT kicked\n");
		return;
	}
	thread_deallocate(th);
	printf("mt6781wdt: kicked every %u s (mtk_wdt_kick=1)\n", KICK_MS / 1000);
}
