/*
 * The buttons of the realme 8i (spaced, MT6781), as a source of button presses for /dev/hakios-touch
 * (sys/hakios.input.h). Only with boot-arg buttons=3: buttons=1 is the POCO's (hakios.buttons.c), buttons=2 the
 * Hot 20's (hakios.mtk_keys.c).
 *
 * Where they are (Android's device tree, the kp@10010000 node; Android's kernel, mt-plat/mt6781 upmu_hw.h):
 *   volume down  the SoC's keypad, key 1 of its map (kpd-hw-init-map 0x72): bit 1 of KP_MEM1, clear while the
 *                key is down -- mtkloader's KEY_PROBE saw it go 0xffff -> 0xfbfd on the phone
 *   volume up    the PMIC's "reset key" (kpd-sw-rstkey 0x73), an MT6358: HOMEKEY_DEB, bit 3 of TOPSTATUS
 *   power        the same PMIC: PWRKEY_DEB, bit 1 of TOPSTATUS
 * TOPSTATUS (0x28) is read through the PMIC wrapper's WACS2 channel (0x1000d000 + 0xc20, as on the MT6768 and
 * MT6785; LK's log on the phone has WACS1 at 0xc10). Its bits are clear while a key is down. Nothing interrupts:
 * a thread looks every 20 ms, and a key has to read the same twice in a row before it counts. Reads only.
 *
 * Boot-arg keylog=1: a line for every change, to check the wiring against.
 */
#include <kern/clock.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.input.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

#define KPD_PHYS        0x10010000
#define KP_MEM1         0x0004          /* keys 0..15, a bit clear while the key is down */

#define PWRAP_PHYS      0x1000d000
#define PWRAP_WACS2_CMD     0xc20
#define PWRAP_WACS2_RDATA   0xc24
#define PWRAP_WACS2_VLDCLR  0xc28
#define PWRAP_FSM(r)        (((r) >> 16) & 7u)
#define PWRAP_FSM_IDLE      0
#define PWRAP_FSM_WFVLDCLR  6

#define MT6358_TOPSTATUS    0x28
#define PWRKEY_DEB          (1u << 1)   /* clear while the power key is down */
#define HOMEKEY_DEB         (1u << 3)   /* clear while volume up is down */

static vm_offset_t kpd, pwrap;

static inline uint32_t
rd(vm_offset_t base, uint32_t at)
{
	return *(volatile uint32_t *)(base + at);
}

static inline void
wr(vm_offset_t base, uint32_t at, uint32_t value)
{
	*(volatile uint32_t *)(base + at) = value;
}

static bool
pwrap_wait(uint32_t state)
{
	for (uint32_t spin = 0; spin < 100000; spin++) {
		if (PWRAP_FSM(rd(pwrap, PWRAP_WACS2_RDATA)) == state) {
			return true;
		}
	}
	return false;
}

/* One PMIC register through the wrapper's own channel: idle, command, wait for the answer, take it. */
static bool
pmic_read(uint32_t reg, uint32_t *value)
{
	if (!pwrap_wait(PWRAP_FSM_IDLE)) {
		return false;
	}
	wr(pwrap, PWRAP_WACS2_CMD, (reg >> 1) << 16);
	if (!pwrap_wait(PWRAP_FSM_WFVLDCLR)) {
		return false;
	}
	*value = rd(pwrap, PWRAP_WACS2_RDATA) & 0xffff;
	wr(pwrap, PWRAP_WACS2_VLDCLR, 1);
	return true;
}

struct key {
	const char *name;
	uint32_t button;                /* HAKIOS_BUTTON_* */
	bool down, seen;                /* what was reported, and what the last look found */
};

static struct key keys[3] = {
	{ "volume up", HAKIOS_BUTTON_VOLUME_UP, false, false },
	{ "volume down", HAKIOS_BUTTON_VOLUME_DOWN, false, false },
	{ "power", HAKIOS_BUTTON_LOCK, false, false },
};

static void
keys_thread(__unused void *parameter, __unused wait_result_t result)
{
	uint32_t log = 0, presses = 0;

	PE_parse_boot_argn("keylog", &log, sizeof(log));
	kpd = ml_io_map(KPD_PHYS, 0x1000);
	pwrap = ml_io_map(PWRAP_PHYS, 0x1000);
	printf("mt6781keys: the buttons are looked at every 20 ms; keypad %#x, PMIC status %s\n",
	    rd(kpd, KP_MEM1), pwrap_wait(PWRAP_FSM_IDLE) ? "reachable" : "NOT REACHABLE");

	for (unsigned tick = 1;; tick++) {
		uint32_t mem = rd(kpd, KP_MEM1), top = PWRKEY_DEB | HOMEKEY_DEB;
		bool now[3];

		(void)pmic_read(MT6358_TOPSTATUS, &top);
		now[0] = !(top & HOMEKEY_DEB);
		now[1] = !(mem & (1u << 1));
		now[2] = !(top & PWRKEY_DEB);

		for (unsigned i = 0; i < 3; i++) {
			struct key *k = &keys[i];

			/* the same twice in a row, or it is a bounce */
			if (now[i] == k->seen && now[i] != k->down) {
				k->down = now[i];
				hakios_touch_post(k->down ? HAKIOS_BUTTON_DOWN : HAKIOS_BUTTON_UP, k->button, 0, 0);
				presses += k->down;
				if (log) {
					printf("mt6781keys: %s %s (keypad %#x, PMIC %#x)\n", k->name, k->down ? "down" : "up", mem, top);
				}
			}
			k->seen = now[i];
		}
		if (tick % 3000 == 0) {
			printf("mt6781keys: so far %u presses; keypad %#x, PMIC status %#x\n", presses, mem, top);
		}
		delay(20000);
	}
}

void
hakios_mt6781_keys_init(void)
{
	uint32_t which = 0;
	thread_t thread;

	if (!PE_parse_boot_argn("buttons", &which, sizeof(which)) || which != 3) {
		return;
	}
	if (kernel_thread_start(keys_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	} else {
		printf("mt6781keys: cannot start the thread that reads the buttons\n");
	}
}
