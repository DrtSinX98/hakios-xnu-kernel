/*
 * What the drivers share, kept apart from any one of them, so that each can be left out of the kernel on its own
 * (drivers/<subsystem>/conf, CONFIG_HAKIOS_*: docs/drivers-config.md).
 *
 *   the locks      of the hardware several drivers take turns at: the PMIC arbiter (the buttons, the flashlight, the
 *                  camera's regulators, the PMIC survey), the always-on processor's mailbox, the shared memory's
 *                  allocation and RPMh's sequencer (the Wi-Fi, audio DSP and modem bring-ups, the touch screen's
 *                  supply), and the SMMU's free match registers (Wi-Fi, sound, camera, modem). They were in
 *                  hakios.buttons.c and hakios.wifi.c, which the others then needed in the kernel to link.
 *   kern.hakios    the sysctl node the drivers hang theirs under (it was hakios.battery.c's)
 *   the list       of the drivers this kernel was built with: sysctl kern.hakios.drivers, and a line in the boot log,
 *                  so that a run says what was in it (hakios.drivers.h is made by tools/hakios-config.py kernel).
 */
#include <kern/locks.h>
#include <kern/startup.h>
#include <libkern/libkern.h>
#include <pexpert/pexpert.h>
#include <sys/sysctl.h>
#include <sys/systm.h>

#include "hakios.drivers.h"

/* kern.hakios: the node every driver of this port hangs its sysctls under (it was hakios.battery.c's) */
SYSCTL_NODE(_kern, OID_AUTO, hakios, CTLFLAG_RW | CTLFLAG_LOCKED, 0, "redwood phone bits");

static LCK_GRP_DECLARE(hakios_platform_lock_group, "hakios-platform");

/*
 * One transfer at a time through the PMIC arbiter for this system, whoever asks: with two at once the arbiter drops
 * one ("the transfer was dropped"), as Linux never lets happen (one lock for the bus). A spin lock: it is taken
 * around a transfer of some tens of microseconds.
 */
static LCK_SPIN_DECLARE(spmi_lock, &hakios_platform_lock_group);

void
hakios_spmi_lock(void)
{
	lck_spin_lock(&spmi_lock);
}

void
hakios_spmi_unlock(void)
{
	lck_spin_unlock(&spmi_lock);
}

/*
 * What the DSPs' starts use while another may be at work beside them: the mailbox to the always-on processor, one
 * message at a time, and the allocation of items in the shared memory (whose hardware lock both take under the same
 * number, so it does not keep the two apart); RPMh's sequencer (TCS 0 of the RSC); a free match register and context
 * of the SMMU, found and taken.
 */
LCK_MTX_DECLARE(hakios_aop_mailbox_lock, &hakios_platform_lock_group);
LCK_MTX_DECLARE(hakios_smem_alloc_lock, &hakios_platform_lock_group);
LCK_MTX_DECLARE(hakios_rsc_lock, &hakios_platform_lock_group);
LCK_MTX_DECLARE(hakios_smmu_lock, &hakios_platform_lock_group);

static char hakios_drivers_string[] = HAKIOS_DRIVERS_LIST;

SYSCTL_STRING(_kern_hakios, OID_AUTO, drivers, CTLFLAG_RD | CTLFLAG_LOCKED, hakios_drivers_string, 0,
    "the drivers this kernel was built with (CONFIG_HAKIOS_* of its configuration, lowercase)");

/*
 * kern.hakios.say (boot-arg hakios_say=1): a line written here by any program goes into the kernel's log with its time
 * (printf: the message buffer and the console), at most 255 bytes. For start-up steps of system programs that a run's
 * console would lose (libhakios in backboardd); without the boot-arg a write is refused.
 */
static int
sysctl_hakios_say SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	static int allowed = -1;
	char line[256];
	size_t n;
	int error;

	if (allowed < 0) {
		int on = 0;

		allowed = PE_parse_boot_argn("hakios_say", &on, sizeof(on)) && on;
	}
	if (req->newptr == USER_ADDR_NULL) {
		return 0;
	}
	if (!allowed) {
		return EPERM;
	}
	n = MIN(req->newlen, sizeof(line) - 1);
	error = SYSCTL_IN(req, line, n);
	if (error != 0) {
		return error;
	}
	line[n] = 0;
	while (n > 0 && (line[n - 1] == '\n' || line[n - 1] == 0)) {
		line[--n] = 0;
	}
	printf("%s\n", line);
	return 0;
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, say, CTLTYPE_STRING | CTLFLAG_WR | CTLFLAG_ANYBODY | CTLFLAG_LOCKED, NULL, 0,
    sysctl_hakios_say, "A", "a line into the kernel's log (boot-arg hakios_say=1)");

static void
hakios_platform_init(void)
{
	printf("hakios-kernel: %d drivers built in (%s): %s\n", HAKIOS_DRIVERS_COUNT, HAKIOS_DRIVERS_CONFIG, hakios_drivers_string);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, hakios_platform_init);
