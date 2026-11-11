/*
 * The realme 8i's UFS (MT6781, ufshci@11270000, UFSHCI 3.1), read only: the partition xnusys of the user unit
 * (LUN 2) as /dev/mtkufs0 and /dev/rmtkufs0, and with mtkufs_root=1 the root (iokit/bsddev/IOKitBSDInit.cpp asks
 * hakios_mtk_ufs_root for it). Boot-arg mtkufs=1, and mtkloader's mtkufs_lba=, mtkufs_blocks=, mtkufs_lkutrl=
 * (XNU_UFS=1: it found xnusys in the GPT through LK's list); without all four nothing here touches the controller.
 *
 * Adapted from drivers/storage/hakios.ufshc.c (the POCO's): its transfer list switch, whitelist, slot layout and
 * the check of what a slot holds, cut down to one request at a time, polled. What differs: the list and command
 * descriptors are this kernel's own memory (one contiguous piece; its cached view written out once and never used
 * again, mapped a second time without caching, as hakios.lx.c does); there is no loader to share the controller with
 * during a run (LK is gone): LK's list address comes from mtkloader, is checked against the controller's at the switch
 * and is put back at the end (hakios_mt6781_ufs_quiesce, from loader_return).
 *
 * LK leaves the controller up: link up, its list running at its own memory, nothing in flight. Nothing of that is
 * changed here: no reset, link startup, UIC command, clock, power, interrupt enable or AHIT write; the task
 * management list is not touched. Written: UTRLRSR, UTRLBA and UTRLBAU at the switches, the doorbell, IS's transfer
 * completion bit (write 1 to clear: LK left IE on, so its line is not left raised), UTRLCNR's bits of this kernel's
 * requests at the end, UTRLCLR for a slot that does not finish.
 *
 * Read only by construction: one check (mufs_refusal) before anything goes into a slot -- READ(10), READ CAPACITY(10),
 * TEST UNIT READY, nothing else; LUN 2 only; data from the device or none, never the W flag; a READ(10)'s blocks
 * inside xnusys's extent, or, only while the start checks that extent against the GPT itself (mufs_gpt_check), inside
 * the primary GPT (blocks 1..5); a disagreement with mtkloader and nothing more is read. What the slot then holds is read back and must pass again before the doorbell. Slot 0 holds
 * the proof (a TEST UNIT READY that must come back in this list after the switch), slot 1 the reads, the others a
 * TEST UNIT READY that is never rung. Data goes through one contiguous bounce buffer, cleaned and invalidated around
 * every read. The doorbell is rung and waited for with interrupts off.
 *
 * Writing (boot-arg mtkufs_write=1, and mtkloader's mtkufs_dlba= mtkufs_dblocks=, checked against the GPT here like
 * xnusys's): WRITE(10) and READ(10) only inside the partition darwin past its first MiB (the boot settings block's), SYNCHRONIZE CACHE(10)
 * of all of it; nothing else ever written. mtkufs_wtest=1 is the dry run: one MiB of a known pattern at darwin + 1 MiB,
 * written, the cache synchronized, read back and compared, with the speed of each.
 *
 * The data volume's store (userland/src/var_store.c, the POCO's protocol): /dev/rufs_darwin, darwin past its first
 * MiB counted in bytes from darwin's start; kern.hakios.var_store_at says where the store starts (1 MiB);
 * kern.hakios.var_busy is the claim a writer must hold; kern.hakios.var_save the handshake with the saver, asked at
 * the end of a run (hakios_var_save_on_return). Every write is read back and compared before it is answered; the
 * cache is synchronized when the device is closed and before LK's list goes back.
 *
 * var_store_crash_at=N, a test only: right after the N-th write to /dev/rufs_darwin of this boot, before its read-back
 * and with nothing synchronized, the phone is reset at once (PSCI SYSTEM_RESET): a save cut off as by a power loss.
 *
 * The data volume (boot-arg mtkufs_data=1 read only, 2 writable; with mtkufs_write=1): /dev/mtkufs_data, a block
 * device over darwin past the store's two slots and the breadcrumbs' MiB (DATA_FIRST) to its end, which init mounts on /private/var as HFS Plus
 * (var_hfs=mtkufs_data). Every I/O is checked against that window here, so it never reaches the store or the boot
 * settings block; the whitelist still stands under it. Its writes are not read back (mtkufs_readback=1: they are);
 * DKIOCSYNCHRONIZE synchronizes the device's cache (HFS Plus's journal asks it); the end of a run syncs the file
 * systems first (hakios_var_save_on_return). mtkufs_data_crash_at=N, a test only: the phone reset right after the
 * N-th write to it, as var_store_crash_at.
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <mach/vm_param.h>
#include <miscfs/devfs/devfs.h>
#include <pexpert/pexpert.h>
#include <stdarg.h>
#include <sys/buf.h>
#include <sys/conf.h>
#include <sys/disk.h>
#include <sys/errno.h>
#include <sys/fcntl.h>
#include <sys/hakios.input.h>
#include <sys/msgbuf.h>
#include <sys/proc.h>
#include <sys/sysctl.h>
#include <sys/systm.h>
#include <sys/uio.h>
#include <sys/vnode.h>
#include <kern/kalloc.h>
#include <vm/vm_kern.h>
#include <vm/memory_types.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern vm_offset_t ml_io_map_unmappable(vm_offset_t phys_addr, vm_size_t size, unsigned int flags);
extern uint64_t kvtophys(vm_offset_t va);
extern void FlushPoC_DcacheRegion(vm_offset_t va, size_t length);
extern boolean_t ml_set_interrupts_enabled(boolean_t enable);
extern void loader_log_putc(char c);
extern void IOSleep(unsigned int milliseconds);
void hakios_mt6781_ufs_quiesce(void);
void hakios_var_save_on_return(void);
extern int sync_internal(void);                 /* bsd/vfs/vfs_syscalls.c: the file systems synced, within 5 s */
static int data_sync(void);
extern void bsd_log_lock_safe(void);           /* bsd/kern/subr_log.c */
extern void bsd_log_unlock(void);

#define MUFS_PHYS           0x11270000ULL       /* ufshci@11270000, "mediatek,mt8183-ufshci" */
#define MUFS_SIZE           0x1000
#define MUFS_LUN            2                   /* the user unit, to LK and to this kernel */

#define UFSHC_CAP           0x00
#define UFSHC_VER           0x08
#define UFSHC_AHIT          0x18
#define UFSHC_IS            0x20
#define UFSHC_IE            0x24
#define UFSHC_HCS           0x30
#define UFSHC_HCE           0x34
#define UFSHC_UTRIACR       0x4c
#define UFSHC_UTRLBA        0x50
#define UFSHC_UTRLBAU       0x54
#define UFSHC_UTRLDBR       0x58
#define UFSHC_UTRLCLR       0x5c
#define UFSHC_UTRLRSR       0x60
#define UFSHC_UTRLCNR       0x64
#define UFSHC_UTMRLBA       0x70
#define UFSHC_UTMRLDBR      0x78
#define UFSHC_UTMRLRSR      0x80

#define HCS_DP              (1u << 0)
#define HCS_UTRLRDY         (1u << 1)
#define IS_UTRCS            (1u << 0)
#define IS_FATAL            ((1u << 2) | (1u << 11) | (1u << 12) | (1u << 16) | (1u << 17))

#define UTRD_CT_MASK        0xf0000000u
#define UTRD_CT_UFS         (1u << 28)
#define UTRD_DD_MASK        (3u << 25)
#define UTRD_DD_NONE        (0u << 25)
#define UTRD_DD_OUT         (1u << 25)          /* host to device: WRITE(10) into darwin's window only */
#define UTRD_DD_IN          (2u << 25)
#define OCS_SUCCESS         0x0
#define OCS_INVALID         0xf

#define UPIU_COMMAND        0x01
#define UPIU_RESPONSE       0x21
#define UPIU_FLAG_READ      0x40
#define UPIU_FLAG_WRITE     0x20

#define SLOTS               32
#define SLOT_PROOF          0
#define SLOT_READ           1
#define UCD_RESPONSE        512                 /* in a command descriptor: the UPIU at 0, the response, the PRD table */
#define UCD_PRDT            1024
#define UCD_SIZE            2048
#define AREA_UCD            4096                /* the list in the first page, the command descriptors after it */
#define AREA_SIZE           (AREA_UCD + SLOTS * UCD_SIZE)
#define BOUNCE              (256u << 10)        /* the most one PRD table entry describes */
#define BLOCK               4096u
#define SECTOR              512u                /* what the block device says its block is (as the Hot 20's) */
#define TIMEOUT_US          1000000
#define GPT_HEADER          1                   /* the primary GPT, as gpt-carve.py lays it for 4096-byte blocks */
#define GPT_ENTRIES         2
#define GPT_ENTRY_BLOCKS    4                   /* 128 entries of 128 bytes */
#define DARWIN_KEEP         256                 /* darwin's first MiB: the boot settings block, never written here */
#define DARWIN_MINOR        1                   /* /dev/rufs_darwin; minor 0 is xnusys */
#define DATA_MINOR          2                   /* /dev/mtkufs_data */
#define CRUMB_FIRST         (DARWIN_KEEP + 2 * 524288)  /* past the store's two 2 GiB slots (var_store.c SLOT_BYTES) */
#define CRUMB_BLOCKS        256                         /* one MiB of breadcrumbs, one block each */
#define DATA_FIRST          (CRUMB_FIRST + CRUMB_BLOCKS)

struct mufs_cmd {
	uint8_t upiu[32];
	uint32_t dw0;
};

static vm_offset_t hc;
static volatile uint32_t *mufs_list;            /* this kernel's list, not cached */
static vm_offset_t mufs_ucd;                    /* the command descriptors, not cached */
static uint64_t mufs_area_pa, mufs_bounce_pa;
static uint8_t *mufs_bounce;                    /* cached; cleaned and invalidated around every read */
static uint64_t part_lba, part_blocks, lk_utrl; /* from mtkloader */
static uint64_t dlba, dblocks;                  /* darwin, from mtkloader; used only with mtkufs_write=1 and the GPT's word */
static bool write_on, darwin_ok;                /* mtkufs_write=1; darwin checked against the GPT */
static uint64_t mufs_writes;
static bool darwin_dirty;                       /* written since the cache was last synchronized */
static uint8_t darwin_buf[BOUNCE];              /* /dev/rufs_darwin's data on its way, under mufs_lock */
static uint32_t darwin_writes, crash_at;        /* this boot's writes to /dev/rufs_darwin; var_store_crash_at=N */
static uint32_t data_mode, data_bsize = BLOCK;  /* mtkufs_data=: 1 read only, 2 writable; the volume's block size */
static uint32_t data_writes, data_crash_at;     /* this boot's writes to /dev/mtkufs_data; mtkufs_data_crash_at=N */
static bool data_readback, data_written;        /* mtkufs_readback=1; written this boot (synced at the end) */
static uint32_t crumbs_on, crumb_next;          /* mtkufs_crumbs=1; the next breadcrumb's block */
static uint64_t crumb_boot;                     /* this boot's counter at its first breadcrumb */
extern uint32_t psci_smp_hvc;                   /* osfmk/arm64/psci_smp.c: the firmware is called with hvc, else smc */
static uint64_t dev_blocks;                     /* what the device says it is: xnusys's APFS containers, else all of it */
static bool apfs_found;
static uint32_t cnr_in;
static bool started, usable, in, dead;
static bool gpt_window;                         /* the primary GPT readable: only inside mufs_gpt_check */
static int bmajor = -1, cmajor = -1;
static uint64_t mufs_reads, mufs_errors, mufs_refused;
static LCK_GRP_DECLARE(mufs_lck_grp, "hakios-mt6781-ufs");
static LCK_MTX_DECLARE(mufs_lock, &mufs_lck_grp);

/* at the end (loader_return), into the loader's log directly: printf is not for that moment */
static void
say_late(const char *format, ...) __printflike(1, 2);
static void
say_late(const char *format, ...)
{
	char line[208];
	va_list list;

	va_start(list, format);
	vsnprintf(line, sizeof(line), format, list);
	va_end(list);
	for (const char *c = line; *c != 0; c++) {
		loader_log_putc(*c);
	}
}

static inline uint32_t
rd(uint32_t reg)
{
	return *(volatile uint32_t *)(hc + reg);
}

static inline void
wr(uint32_t reg, uint32_t value)
{
	__asm__ volatile ("dsb sy" ::: "memory");
	*(volatile uint32_t *)(hc + reg) = value;
	__asm__ volatile ("dsb sy" ::: "memory");
}

static uint64_t
us_since(uint64_t start)
{
	uint64_t ns;

	absolutetime_to_nanoseconds(mach_absolute_time() - start, &ns);
	return ns / 1000;
}

static uint32_t
be32(const uint8_t *p)
{
	return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}

/* ---- the whitelist: the one check every command passes before it goes into a slot ---- */

static const char *
mufs_refusal(const uint8_t *upiu, uint32_t dw0)
{
	const uint8_t *cdb = upiu + 16;
	uint32_t dd = dw0 & UTRD_DD_MASK, expected = be32(upiu + 12);

	bool writing = write_on && darwin_ok && cdb[0] == 0x2a;

	if ((dw0 & UTRD_CT_MASK) != UTRD_CT_UFS) {
		return "not a UFS storage command type";
	}
	if (dd == UTRD_DD_MASK || (dd == UTRD_DD_OUT && !writing)) {
		return "data from host to device";
	}
	if ((upiu[0] & 0x3f) != UPIU_COMMAND) {
		return "not a SCSI command (a query, task management, NOP, ...)";
	}
	if ((upiu[1] & UPIU_FLAG_WRITE) && !writing) {
		return "the W flag";
	}
	if (upiu[4] & 0x0f) {
		return "not the SCSI command set";
	}
	if (upiu[2] != MUFS_LUN) {
		return "a logical unit other than 2";
	}
	switch (cdb[0]) {
	case 0x28: {    /* READ(10): inside xnusys */
		uint64_t lba = be32(cdb + 2), n = (uint64_t)cdb[7] << 8 | cdb[8];

		if (dd != UTRD_DD_IN || (upiu[1] & UPIU_FLAG_READ) == 0) {
			return "a read without data from the device";
		}
		if (n == 0 || expected != n * BLOCK) {
			return "a read whose length is not what the UPIU expects";
		}
		if (gpt_window && lba >= GPT_HEADER && lba + n <= GPT_ENTRIES + GPT_ENTRY_BLOCKS) {
			return NULL;
		}
		if (write_on && darwin_ok && lba >= dlba + DARWIN_KEEP && lba - dlba < dblocks && n <= dblocks - (lba - dlba)) {
			return NULL;                    /* darwin's window */
		}
		if (lba < part_lba || lba - part_lba >= part_blocks || n > part_blocks - (lba - part_lba)) {
			return "a read outside xnusys";
		}
		return NULL;
	}
	case 0x2a: {    /* WRITE(10): inside darwin past its first MiB, with mtkufs_write=1 only */
		uint64_t lba = be32(cdb + 2), n = (uint64_t)cdb[7] << 8 | cdb[8];

		if (!writing) {
			return "a write (mtkufs_write=1 and a checked darwin needed)";
		}
		if (dd != UTRD_DD_OUT || (upiu[1] & (UPIU_FLAG_READ | UPIU_FLAG_WRITE)) != UPIU_FLAG_WRITE) {
			return "a write without data to the device";
		}
		if (cdb[1] != 0 || cdb[6] != 0 || cdb[9] != 0) {
			return "a write with flags, group or control set";
		}
		if (n == 0 || expected != n * BLOCK) {
			return "a write whose length is not what the UPIU expects";
		}
		if (lba < dlba + DARWIN_KEEP || lba - dlba >= dblocks || n > dblocks - (lba - dlba)) {
			return "a write outside darwin's window";
		}
		return NULL;
	}
	case 0x35:      /* SYNCHRONIZE CACHE(10): all of the cache, with mtkufs_write=1 only */
		if (!write_on || !darwin_ok) {
			return "SYNCHRONIZE CACHE(10) without mtkufs_write=1";
		}
		return dd == UTRD_DD_NONE && expected == 0 && (upiu[1] & (UPIU_FLAG_READ | UPIU_FLAG_WRITE)) == 0 &&
		       cdb[1] == 0 && be32(cdb + 2) == 0 && cdb[7] == 0 && cdb[8] == 0 ? NULL :
		       "SYNCHRONIZE CACHE(10) other than of the whole cache, without data";
	case 0x25:      /* READ CAPACITY(10) */
		return dd == UTRD_DD_IN && (upiu[1] & UPIU_FLAG_READ) && expected == 8 ? NULL :
		       "READ CAPACITY(10) other than 8 bytes from the device";
	case 0x00:      /* TEST UNIT READY */
		return dd == UTRD_DD_NONE && expected == 0 && (upiu[1] & UPIU_FLAG_READ) == 0 ? NULL :
		       "TEST UNIT READY with data";
	default:
		return "a SCSI operation not on the list";
	}
}

static bool
mufs_permitted(const uint8_t *upiu, uint32_t dw0)
{
	const char *why = mufs_refusal(upiu, dw0);

	if (why == NULL) {
		return true;
	}
	mufs_refused++;
	printf("mtkufs: REFUSED (%s): UPIU %02x %02x %02x, CDB %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x, "
	    "descriptor %08x; it goes into no slot\n", why, upiu[0], upiu[1], upiu[2], upiu[16], upiu[17], upiu[18],
	    upiu[19], upiu[20], upiu[21], upiu[22], upiu[23], upiu[24], upiu[25], dw0);
	return false;
}

static void
make_read(struct mufs_cmd *c, uint32_t lba, uint32_t blocks)
{
	uint32_t bytes = blocks * BLOCK;

	bzero(c, sizeof(*c));
	c->upiu[0] = UPIU_COMMAND;
	c->upiu[1] = UPIU_FLAG_READ;
	c->upiu[2] = MUFS_LUN;
	c->upiu[3] = SLOT_READ;
	c->upiu[12] = (uint8_t)(bytes >> 24);
	c->upiu[13] = (uint8_t)(bytes >> 16);
	c->upiu[14] = (uint8_t)(bytes >> 8);
	c->upiu[15] = (uint8_t)bytes;
	c->upiu[16] = 0x28;
	c->upiu[18] = (uint8_t)(lba >> 24);
	c->upiu[19] = (uint8_t)(lba >> 16);
	c->upiu[20] = (uint8_t)(lba >> 8);
	c->upiu[21] = (uint8_t)lba;
	c->upiu[23] = (uint8_t)(blocks >> 8);
	c->upiu[24] = (uint8_t)blocks;
	c->dw0 = UTRD_CT_UFS | UTRD_DD_IN;              /* no interrupt, no inline encryption */
}

static void
make_tur(struct mufs_cmd *c, uint8_t tag)
{
	bzero(c, sizeof(*c));
	c->upiu[0] = UPIU_COMMAND;
	c->upiu[2] = MUFS_LUN;
	c->upiu[3] = tag;
	c->dw0 = UTRD_CT_UFS | UTRD_DD_NONE;
}

static void
make_capacity(struct mufs_cmd *c)
{
	bzero(c, sizeof(*c));
	c->upiu[0] = UPIU_COMMAND;
	c->upiu[1] = UPIU_FLAG_READ;
	c->upiu[2] = MUFS_LUN;
	c->upiu[3] = SLOT_READ;
	c->upiu[15] = 8;
	c->upiu[16] = 0x25;
	c->dw0 = UTRD_CT_UFS | UTRD_DD_IN;
}

static void
make_write(struct mufs_cmd *c, uint32_t lba, uint32_t blocks)
{
	make_read(c, lba, blocks);
	c->upiu[1] = UPIU_FLAG_WRITE;
	c->upiu[16] = 0x2a;
	c->dw0 = UTRD_CT_UFS | UTRD_DD_OUT;
}

static void
make_sync(struct mufs_cmd *c)
{
	bzero(c, sizeof(*c));
	c->upiu[0] = UPIU_COMMAND;
	c->upiu[2] = MUFS_LUN;
	c->upiu[3] = SLOT_READ;
	c->upiu[16] = 0x35;
	c->dw0 = UTRD_CT_UFS | UTRD_DD_NONE;
}

/* The whitelist tried on commands made up here and never sent. Pure bytes; nothing touches the controller. */
static bool
mufs_selftest(void)
{
	static const struct {
		uint8_t op, flags, lun, type;
		int64_t at;             /* the READ(10)'s first block, from xnusys's start (-1 before it, -4 the GPT header) */
		int32_t blocks;         /* its length; <0: from xnusys's end */
		uint32_t dd;
		bool pass;
		const char *what;
	} cases[] = {
		{ 0x28, UPIU_FLAG_READ, 2, UPIU_COMMAND, 0, 1, UTRD_DD_IN, true, "READ(10) of xnusys's first block" },
		{ 0x28, UPIU_FLAG_READ, 2, UPIU_COMMAND, 0, 64, UTRD_DD_IN, true, "READ(10) of 64 blocks at its start" },
		{ 0x28, UPIU_FLAG_READ, 2, UPIU_COMMAND, -2, 1, UTRD_DD_IN, true, "READ(10) of its last block" },
		{ 0x28, UPIU_FLAG_READ, 2, UPIU_COMMAND, -1, 1, UTRD_DD_IN, false, "READ(10) of the block before it" },
		{ 0x28, UPIU_FLAG_READ, 2, UPIU_COMMAND, -2, 2, UTRD_DD_IN, false, "READ(10) across its end" },
		{ 0x28, UPIU_FLAG_READ, 2, UPIU_COMMAND, -3, 1, UTRD_DD_IN, false, "READ(10) of the block after it" },
		{ 0x28, UPIU_FLAG_READ, 2, UPIU_COMMAND, -4, 1, UTRD_DD_IN, false, "READ(10) of the GPT header outside its check" },
		{ 0x28, UPIU_FLAG_READ, 0, UPIU_COMMAND, 0, 1, UTRD_DD_IN, false, "READ(10) of LUN 0" },
		{ 0x28, UPIU_FLAG_READ, 3, UPIU_COMMAND, 0, 1, UTRD_DD_IN, false, "READ(10) of LUN 3" },
		{ 0x28, UPIU_FLAG_READ, 0xc4, UPIU_COMMAND, 0, 1, UTRD_DD_IN, false, "READ(10) of the RPMB unit" },
		{ 0x28, UPIU_FLAG_READ | UPIU_FLAG_WRITE, 2, UPIU_COMMAND, 0, 1, UTRD_DD_IN, false, "READ(10) with the W flag" },
		{ 0x28, UPIU_FLAG_READ, 2, UPIU_COMMAND, 0, 1, UTRD_DD_OUT, false, "READ(10) with data host to device" },
		{ 0x28, UPIU_FLAG_READ, 2, UPIU_COMMAND, 0, 1, UTRD_DD_MASK, false, "READ(10) with direction 3" },
		{ 0x2a, UPIU_FLAG_WRITE, 2, UPIU_COMMAND, 0, 1, UTRD_DD_OUT, false, "WRITE(10)" },
		{ 0x2a, UPIU_FLAG_READ, 2, UPIU_COMMAND, 0, 1, UTRD_DD_IN, false, "WRITE(10) dressed as a read" },
		{ 0x88, UPIU_FLAG_READ, 2, UPIU_COMMAND, 0, 1, UTRD_DD_IN, false, "READ(16)" },
		{ 0x42, 0, 2, UPIU_COMMAND, 0, 0, UTRD_DD_NONE, false, "UNMAP" },
		{ 0x12, UPIU_FLAG_READ, 2, UPIU_COMMAND, 0, 0, UTRD_DD_IN, false, "INQUIRY" },
		{ 0x03, UPIU_FLAG_READ, 2, UPIU_COMMAND, 0, 0, UTRD_DD_IN, false, "REQUEST SENSE" },
		{ 0x35, 0, 2, UPIU_COMMAND, 0, 0, UTRD_DD_NONE, false, "SYNCHRONIZE CACHE(10)" },
		{ 0x1b, 0, 2, UPIU_COMMAND, 0, 0, UTRD_DD_NONE, false, "START STOP UNIT" },
		{ 0x00, 0, 2, 0x16, 0, 0, UTRD_DD_NONE, false, "a QUERY REQUEST" },
		{ 0x00, 0, 2, 0x00, 0, 0, UTRD_DD_NONE, false, "NOP OUT" },
		{ 0x00, 0, 2, UPIU_COMMAND, 0, 0, UTRD_DD_IN, false, "TEST UNIT READY with data in" },
	};
	struct mufs_cmd c;
	bool right = true;

	for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
		uint64_t lba = cases[i].at == -1 ? part_lba - 1 : cases[i].at == -2 ? part_lba + part_blocks - 1 :
		    cases[i].at == -3 ? part_lba + part_blocks : cases[i].at == -4 ? GPT_HEADER : part_lba + (uint64_t)cases[i].at;
		uint32_t n = cases[i].blocks > 0 ? (uint32_t)cases[i].blocks : 1;
		bool passed;

		make_read(&c, (uint32_t)lba, n);
		c.upiu[0] = cases[i].type;
		c.upiu[1] = cases[i].flags;
		c.upiu[2] = cases[i].lun;
		c.upiu[16] = cases[i].op;
		if (cases[i].blocks == 0) {
			bzero(c.upiu + 12, 4);
			bzero(c.upiu + 17, 15);
		}
		c.dw0 = UTRD_CT_UFS | cases[i].dd;
		passed = mufs_refusal(c.upiu, c.dw0) == NULL;
		if (passed != cases[i].pass) {
			printf("mtkufs: the whitelist %s %s\n", passed ? "LETS THROUGH" : "refuses", cases[i].what);
			right = false;
		}
	}
	/* and what this file makes */
	make_tur(&c, 0);
	right &= mufs_refusal(c.upiu, c.dw0) == NULL;
	make_capacity(&c);
	right &= mufs_refusal(c.upiu, c.dw0) == NULL;
	printf("mtkufs: whitelist tried on %u commands made up and never sent (reads in and around xnusys, other units, "
	    "W flag, data out, WRITE, READ(16), UNMAP, INQUIRY, REQUEST SENSE, SYNC, START STOP, query, NOP): %s\n",
	    (unsigned)(sizeof(cases) / sizeof(cases[0])) + 2, right ? "as it should" : "WRONG");
	return right;
}

/* With darwin open for writing: the same, for the writes and their window (darwin and xnusys as they are now). */
static bool
mufs_selftest_darwin(void)
{
	static const struct {
		uint8_t op, flags;
		int where;              /* 0 darwin + 1 MiB, 1 darwin's last block, 2 darwin's first, 3 the block before
		                         * darwin + 1 MiB, 4 darwin's end (past it), 5 xnusys's first, 6 block 1 (GPT) */
		uint32_t blocks, dd;
		uint8_t cdb1;
		bool pass;
		const char *what;
	} cases[] = {
		{ 0x2a, UPIU_FLAG_WRITE, 0, 1, UTRD_DD_OUT, 0, true, "WRITE(10) of darwin + 1 MiB" },
		{ 0x2a, UPIU_FLAG_WRITE, 0, 64, UTRD_DD_OUT, 0, true, "WRITE(10) of 64 blocks there" },
		{ 0x2a, UPIU_FLAG_WRITE, 1, 1, UTRD_DD_OUT, 0, true, "WRITE(10) of darwin's last block" },
		{ 0x2a, UPIU_FLAG_WRITE, 1, 2, UTRD_DD_OUT, 0, false, "WRITE(10) across darwin's end" },
		{ 0x2a, UPIU_FLAG_WRITE, 4, 1, UTRD_DD_OUT, 0, false, "WRITE(10) past darwin" },
		{ 0x2a, UPIU_FLAG_WRITE, 2, 1, UTRD_DD_OUT, 0, false, "WRITE(10) of darwin's first block (the boot settings block)" },
		{ 0x2a, UPIU_FLAG_WRITE, 3, 1, UTRD_DD_OUT, 0, false, "WRITE(10) of the last block of the boot settings block" },
		{ 0x2a, UPIU_FLAG_WRITE, 3, 2, UTRD_DD_OUT, 0, false, "WRITE(10) across into the boot settings block" },
		{ 0x2a, UPIU_FLAG_WRITE, 5, 1, UTRD_DD_OUT, 0, false, "WRITE(10) of xnusys" },
		{ 0x2a, UPIU_FLAG_WRITE, 6, 1, UTRD_DD_OUT, 0, false, "WRITE(10) of the GPT" },
		{ 0x2a, UPIU_FLAG_READ, 0, 1, UTRD_DD_OUT, 0, false, "WRITE(10) with the R flag" },
		{ 0x2a, UPIU_FLAG_WRITE, 0, 1, UTRD_DD_IN, 0, false, "WRITE(10) with data from the device" },
		{ 0x2a, UPIU_FLAG_WRITE, 0, 1, UTRD_DD_OUT, 0x08, false, "WRITE(10) with FUA" },
		{ 0x28, UPIU_FLAG_READ, 0, 1, UTRD_DD_IN, 0, true, "READ(10) of darwin + 1 MiB" },
		{ 0x28, UPIU_FLAG_READ, 2, 1, UTRD_DD_IN, 0, false, "READ(10) of the boot settings block" },
		{ 0x28, UPIU_FLAG_READ | UPIU_FLAG_WRITE, 0, 1, UTRD_DD_IN, 0, false, "READ(10) of darwin with the W flag" },
		{ 0x28, UPIU_FLAG_READ, 0, 1, UTRD_DD_OUT, 0, false, "READ(10) of darwin with data to the device" },
		{ 0x8a, UPIU_FLAG_WRITE, 0, 1, UTRD_DD_OUT, 0, false, "WRITE(16)" },
		{ 0x2e, UPIU_FLAG_WRITE, 0, 1, UTRD_DD_OUT, 0, false, "WRITE AND VERIFY(10)" },
		{ 0x42, UPIU_FLAG_WRITE, 0, 1, UTRD_DD_OUT, 0, false, "UNMAP" },
	};
	struct mufs_cmd c;
	bool right = true;

	for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
		uint64_t lba = cases[i].where == 0 ? dlba + DARWIN_KEEP : cases[i].where == 1 ? dlba + dblocks - 1 :
		    cases[i].where == 2 ? dlba : cases[i].where == 3 ? dlba + DARWIN_KEEP - 1 :
		    cases[i].where == 4 ? dlba + dblocks : cases[i].where == 5 ? part_lba : GPT_HEADER;
		bool passed;

		make_write(&c, (uint32_t)lba, cases[i].blocks);
		c.upiu[1] = cases[i].flags;
		c.upiu[16] = cases[i].op;
		c.upiu[17] = cases[i].cdb1;
		c.dw0 = UTRD_CT_UFS | cases[i].dd;
		passed = mufs_refusal(c.upiu, c.dw0) == NULL;
		if (passed != cases[i].pass) {
			printf("mtkufs: the whitelist %s %s\n", passed ? "LETS THROUGH" : "refuses", cases[i].what);
			right = false;
		}
	}
	make_sync(&c);
	right &= mufs_refusal(c.upiu, c.dw0) == NULL;
	c.upiu[17] = 0x02;                      /* IMMED */
	right &= mufs_refusal(c.upiu, c.dw0) != NULL;
	make_sync(&c);
	c.upiu[21] = 1;                         /* a range of blocks */
	right &= mufs_refusal(c.upiu, c.dw0) != NULL;
	printf("mtkufs: whitelist for darwin tried on %u commands made up and never sent (writes in, at the edges of and "
	    "outside its window, the boot settings block, xnusys, the GPT, wrong flags and directions, FUA, WRITE(16), WRITE AND VERIFY, "
	    "UNMAP, SYNCHRONIZE CACHE whole, IMMED and ranged): %s\n", (unsigned)(sizeof(cases) / sizeof(cases[0])) + 3,
	    right ? "as it should" : "WRONG");
	return right;
}

/* ---- one command in one slot ---- */

static volatile uint32_t *
slot_ucd(uint32_t slot)
{
	return (volatile uint32_t *)(mufs_ucd + slot * UCD_SIZE);
}

/* Into the slot: only a command the whitelist passes, and only if what the slot then holds is the same and passes again. */
static int
mufs_place(uint32_t slot, const struct mufs_cmd *c, uint32_t bytes)
{
	volatile uint32_t *ucd = slot_ucd(slot), *utrd = mufs_list + 8 * slot;
	uint64_t ucd_pa = mufs_area_pa + AREA_UCD + slot * UCD_SIZE;
	uint32_t back[8];

	if (bytes > BOUNCE || !mufs_permitted(c->upiu, c->dw0)) {
		return EPERM;
	}
	for (uint32_t i = 0; i < 8; i++) {
		uint32_t w;

		memcpy(&w, c->upiu + 4 * i, 4);
		ucd[i] = w;
	}
	for (uint32_t i = 8; i < (UCD_PRDT + 16) / 4; i++) {
		ucd[i] = 0;
	}
	if (bytes != 0) {
		volatile uint32_t *prd = ucd + UCD_PRDT / 4;

		prd[0] = (uint32_t)mufs_bounce_pa;
		prd[1] = (uint32_t)(mufs_bounce_pa >> 32);
		prd[2] = 0;
		prd[3] = bytes - 1;
	}
	utrd[0] = c->dw0;
	utrd[1] = 0;
	utrd[2] = OCS_INVALID;
	utrd[3] = 0;
	utrd[4] = (uint32_t)ucd_pa;
	utrd[5] = (uint32_t)(ucd_pa >> 32);
	utrd[6] = ((UCD_RESPONSE / 4) << 16) | (UCD_RESPONSE / 4);     /* response: offset (31:16), length (15:0), dwords */
	utrd[7] = ((UCD_PRDT / 4) << 16) | (bytes != 0 ? 1 : 0);      /* PRD table: offset, entries */
	__asm__ volatile ("dsb sy" ::: "memory");
	for (int i = 0; i < 8; i++) {
		back[i] = ucd[i];
	}
	if (memcmp(back, c->upiu, sizeof(back)) != 0 || utrd[0] != c->dw0 ||
	    !mufs_permitted((const uint8_t *)back, utrd[0])) {
		printf("mtkufs: slot %u does not hold what was put in it: not rung\n", slot);
		mufs_refused++;
		return EPERM;
	}
	return 0;
}

/*
 * The doorbell, and the wait for it, with interrupts off: only while this kernel's list is in and running and the
 * slot free. Then the status: OCS, the response UPIU (TEST UNIT READY may answer CHECK CONDITION, a unit attention).
 */
static int
mufs_ring(uint32_t slot, const char *what)
{
	volatile const uint8_t *r = (volatile const uint8_t *)slot_ucd(slot) + UCD_RESPONSE;
	volatile uint32_t *utrd = mufs_list + 8 * slot;
	uint32_t bit = 1u << slot, ocs, db = 0;
	uint64_t t;
	boolean_t istate;
	bool ok, done = false;

	istate = ml_set_interrupts_enabled(FALSE);
	ok = in && !dead && rd(UFSHC_UTRLBA) == (uint32_t)mufs_area_pa && rd(UFSHC_UTRLBAU) == (uint32_t)(mufs_area_pa >> 32) &&
	    (rd(UFSHC_UTRLRSR) & 1) && (rd(UFSHC_UTRLDBR) & bit) == 0;
	if (ok) {
		__asm__ volatile ("dsb sy" ::: "memory");       /* the descriptor, UPIU and PRD table out before the doorbell */
		wr(UFSHC_UTRLDBR, bit);
		for (t = mach_absolute_time(); ((db = rd(UFSHC_UTRLDBR)) & bit) && us_since(t) < TIMEOUT_US;) {
			__builtin_arm_yield();
		}
		done = (db & bit) == 0;
		if (!done) {
			wr(UFSHC_UTRLCLR, ~bit);                /* 0 in the slot's bit clears it */
			dead = true;
		}
		wr(UFSHC_IS, IS_UTRCS);
	}
	ml_set_interrupts_enabled(istate);
	__asm__ volatile ("dsb sy" ::: "memory");
	if (!ok) {
		printf("mtkufs: slot %u (%s) not rung: this list not in, not running, or the slot busy\n", slot, what);
		return ENXIO;
	}
	if (!done) {
		printf("mtkufs: slot %u (%s) not done in %u ms (doorbell %08x, IS %08x, HCS %08x): cleared, nothing more is "
		    "sent\n", slot, what, TIMEOUT_US / 1000, rd(UFSHC_UTRLDBR), rd(UFSHC_IS), rd(UFSHC_HCS));
		return ETIMEDOUT;
	}
	ocs = utrd[2] & 0xff;
	if (ocs != OCS_SUCCESS || (r[0] & 0x3f) != UPIU_RESPONSE || r[3] != slot || r[6] != 0 ||
	    (r[7] != 0 && !(slot == SLOT_PROOF && r[7] == 0x02))) {
		printf("mtkufs: slot %u (%s): OCS %x, response %02x tag %02x, response %02x status %02x, sense key %x asc %02x "
		    "ascq %02x\n", slot, what, ocs, r[0], r[3], r[6], r[7], r[34 + 2] & 0x0f, r[34 + 12], r[34 + 13]);
		return EIO;
	}
	return 0;
}

/* ---- the switches ---- */

/* LK's list back: what is in flight waited for (1 s), cleared if it does not finish, stopped, LK's address, started. */
static bool
mufs_put_back(uint32_t *db_left, uint32_t *cnr_ours)
{
	uint64_t t;

	for (t = mach_absolute_time(); (*db_left = rd(UFSHC_UTRLDBR)) != 0 && us_since(t) < 1000000;) {
		__builtin_arm_yield();
	}
	if (*db_left != 0) {
		wr(UFSHC_UTRLCLR, 0);
		for (t = mach_absolute_time(); rd(UFSHC_UTRLDBR) != 0 && us_since(t) < 1000000;) {
			__builtin_arm_yield();
		}
	}
	*cnr_ours = rd(UFSHC_UTRLCNR) & ~cnr_in;
	if (*cnr_ours != 0) {
		wr(UFSHC_UTRLCNR, *cnr_ours);
	}
	wr(UFSHC_UTRLRSR, 0);
	for (t = mach_absolute_time(); (rd(UFSHC_UTRLRSR) & 1) && us_since(t) < 5000;) {
		__builtin_arm_yield();
	}
	wr(UFSHC_UTRLBA, (uint32_t)lk_utrl);
	wr(UFSHC_UTRLBAU, (uint32_t)(lk_utrl >> 32));
	wr(UFSHC_UTRLRSR, 1);
	in = false;
	return rd(UFSHC_UTRLBA) == (uint32_t)lk_utrl && rd(UFSHC_UTRLBAU) == (uint32_t)(lk_utrl >> 32) &&
	       (rd(UFSHC_UTRLRSR) & 1);
}

/* This kernel's list in, in place of LK's; then the proof: slot 0's TEST UNIT READY must come back in this list. */
static int
mufs_switch_in(void)
{
	uint32_t lo = rd(UFSHC_UTRLBA), hi = rd(UFSHC_UTRLBAU), db = rd(UFSHC_UTRLDBR), rsr = rd(UFSHC_UTRLRSR);
	uint32_t hcs = rd(UFSHC_HCS), is = rd(UFSHC_IS), left, cnr;
	const char *no = NULL;
	boolean_t istate;
	uint64_t t;
	int e;

	if ((rd(UFSHC_HCE) & 1) == 0) {
		no = "the controller is not enabled";
	} else if ((hcs & (HCS_DP | HCS_UTRLRDY)) != (HCS_DP | HCS_UTRLRDY)) {
		no = "no device present or the transfer list not ready";
	} else if ((rsr & 1) == 0) {
		no = "the transfer list is not running";
	} else if (db != 0) {
		no = "a request in flight";
	} else if (is & IS_FATAL) {
		no = "an error status";
	} else if (((uint64_t)hi << 32 | lo) != lk_utrl) {
		no = "the controller's list is not the one mtkloader saw as LK's";
	}
	if (no == NULL) {
		istate = ml_set_interrupts_enabled(FALSE);
		cnr_in = rd(UFSHC_UTRLCNR);
		wr(UFSHC_UTRLRSR, 0);
		for (t = mach_absolute_time(); (rd(UFSHC_UTRLRSR) & 1) && us_since(t) < 5000;) {
			__builtin_arm_yield();
		}
		if (rd(UFSHC_UTRLRSR) & 1) {
			no = "the list did not stop";
			wr(UFSHC_UTRLRSR, 1);                   /* (LK's list, running, as it was) */
		} else {
			wr(UFSHC_UTRLBA, (uint32_t)mufs_area_pa);
			wr(UFSHC_UTRLBAU, (uint32_t)(mufs_area_pa >> 32));
			wr(UFSHC_UTRLRSR, 1);
			in = rd(UFSHC_UTRLBA) == (uint32_t)mufs_area_pa && rd(UFSHC_UTRLBAU) == (uint32_t)(mufs_area_pa >> 32) &&
			    (rd(UFSHC_UTRLRSR) & 1);
			if (!in) {
				no = "the controller did not take this list";
				(void)mufs_put_back(&left, &cnr);
			}
		}
		ml_set_interrupts_enabled(istate);
	}
	if (no != NULL) {
		printf("mtkufs: this kernel's list not put in: %s (list %08x:%08x, mtkloader's note of LK's %llx, doorbell %08x, "
		    "run %08x, HCS %08x, IS %08x)\n", no, hi, lo, lk_utrl, db, rsr, hcs, is);
		return EIO;
	}
	printf("mtkufs: transfer list switched from LK's %08x:%08x to this kernel's %llx (completion %08x)\n", hi, lo,
	    mufs_area_pa, cnr_in);
	e = mufs_ring(SLOT_PROOF, "the proof's TEST UNIT READY");
	if (e != 0) {
		printf("mtkufs: proof FAILED (%d): slot 0 did not come back in this list; LK's list goes back\n", e);
		istate = ml_set_interrupts_enabled(FALSE);
		(void)mufs_put_back(&left, &cnr);
		ml_set_interrupts_enabled(istate);
		dead = true;
		return e;
	}
	printf("mtkufs: proof: TEST UNIT READY in this list's slot 0 came back here (status %02x)\n",
	    ((volatile const uint8_t *)slot_ucd(SLOT_PROOF))[UCD_RESPONSE + 7]);
	return 0;
}

/* ---- reading ---- */

/* blocks [lba, lba + blocks) of LUN 2 into the bounce buffer (mufs_lock held) */
static int
read_abs(uint64_t lba, uint32_t blocks, const char *what)
{
	struct mufs_cmd c;
	int error;

	make_read(&c, (uint32_t)lba, blocks);
	/* nothing of the buffer left in a cache to be written back over what the controller writes */
	FlushPoC_DcacheRegion((vm_offset_t)mufs_bounce, (size_t)blocks * BLOCK);
	error = mufs_place(SLOT_READ, &c, blocks * BLOCK);
	if (error == 0) {
		error = mufs_ring(SLOT_READ, what);
	}
	/* and nothing read before the controller wrote it */
	FlushPoC_DcacheRegion((vm_offset_t)mufs_bounce, (size_t)blocks * BLOCK);
	return error;
}

/* blocks [lba, lba + blocks) of LUN 2 from src, through the bounce buffer (mufs_lock held; the whitelist keeps it in
 * darwin's window) */
static int
write_abs(uint64_t lba, uint32_t blocks, const uint8_t *src, const char *what)
{
	struct mufs_cmd c;
	int error;

	memcpy(mufs_bounce, src, (size_t)blocks * BLOCK);
	/* out of the cache to memory, where the controller reads it */
	FlushPoC_DcacheRegion((vm_offset_t)mufs_bounce, (size_t)blocks * BLOCK);
	make_write(&c, (uint32_t)lba, blocks);
	error = mufs_place(SLOT_READ, &c, blocks * BLOCK);
	if (error == 0) {
		error = mufs_ring(SLOT_READ, what);
	}
	if (error == 0) {
		mufs_writes++;
	}
	return error;
}

/* the device's write cache out to the flash (mufs_lock held) */
static int
mufs_sync(void)
{
	struct mufs_cmd c;
	int error;

	make_sync(&c);
	error = mufs_place(SLOT_READ, &c, 0);
	return error != 0 ? error : mufs_ring(SLOT_READ, "SYNCHRONIZE CACHE(10)");
}

/* bytes [offset, offset + len) of xnusys into dst, through the bounce buffer, a slot's worth at a time */
static int
mufs_read(uint64_t offset, uint8_t *dst, uint64_t len)
{
	int error = 0;

	if (!usable || dead || offset > part_blocks * BLOCK || len > part_blocks * BLOCK - offset) {
		return ENXIO;
	}
	lck_mtx_lock(&mufs_lock);
	while (len > 0 && error == 0) {
		uint64_t first = offset / BLOCK, skip = offset % BLOCK, blocks = (skip + len + BLOCK - 1) / BLOCK, take;
		char what[48];

		if (blocks > BOUNCE / BLOCK) {
			blocks = BOUNCE / BLOCK;
		}
		take = blocks * BLOCK - skip;
		if (take > len) {
			take = len;
		}
		snprintf(what, sizeof(what), "xnusys blocks %llu..%llu", first, first + blocks - 1);
		error = read_abs(part_lba + first, (uint32_t)blocks, what);
		if (error == 0) {
			memcpy(dst, mufs_bounce + skip, take);
			mufs_reads++;
			offset += take;
			dst += take;
			len -= take;
		} else {
			mufs_errors++;
		}
	}
	lck_mtx_unlock(&mufs_lock);
	return error;
}

static uint64_t
le(const uint8_t *p, int n)
{
	uint64_t v = 0;

	for (int i = n - 1; i >= 0; i--) {
		v = v << 8 | p[i];
	}
	return v;
}

/*
 * xnusys's extent as the GPT itself has it (header and entries, both CRCs checked), against mtkloader's: the reads
 * are not confined to xnusys until both agree.
 */
static bool
mufs_gpt_check(void)
{
	static uint8_t hdr[BLOCK];
	static const char name[] = "xnusys", dname[] = "darwin";
	const uint8_t *ent = mufs_bounce;
	const char *no = NULL;
	uint64_t entries = 0, first = 0, last = 0, dfirst = 0, dlast = 0;
	uint32_t hsize = 0, hcrc, count = 0, esize = 0, ecrc;
	int e;

	lck_mtx_lock(&mufs_lock);
	gpt_window = true;
	e = read_abs(GPT_HEADER, 1, "the GPT header");
	if (e == 0) {
		memcpy(hdr, mufs_bounce, BLOCK);
		e = read_abs(GPT_ENTRIES, GPT_ENTRY_BLOCKS, "the GPT entries");
	}
	gpt_window = false;
	if (e != 0) {
		no = "the GPT could not be read";
	} else if (memcmp(hdr, "EFI PART", 8) != 0) {
		no = "no GPT signature at block 1";
	} else {
		hsize = (uint32_t)le(hdr + 12, 4);
		hcrc = (uint32_t)le(hdr + 16, 4);
		entries = le(hdr + 72, 8);
		count = (uint32_t)le(hdr + 80, 4);
		esize = (uint32_t)le(hdr + 84, 4);
		ecrc = (uint32_t)le(hdr + 88, 4);
		if (hsize < 92 || hsize > BLOCK) {
			no = "a header size out of range";
		} else {
			bzero(hdr + 16, 4);
			if (crc32(0, hdr, hsize) != hcrc) {
				no = "the header's CRC is wrong";
			}
		}
		if (no == NULL && (entries != GPT_ENTRIES || esize != 128 || count == 0 || count * esize > GPT_ENTRY_BLOCKS * BLOCK)) {
			no = "the entries are not at block 2, 128 bytes each, in 4 blocks";
		} else if (no == NULL && crc32(0, ent, count * esize) != ecrc) {
			no = "the entries' CRC is wrong";
		}
		for (uint32_t i = 0; no == NULL && i < count; i++) {
			const uint8_t *x = ent + i * esize;
			uint32_t k;

			for (k = 0; name[k] && x[56 + 2 * k] == (uint8_t)name[k] && x[57 + 2 * k] == 0; k++) {
			}
			if (name[k] == 0 && x[56 + 2 * k] == 0 && x[57 + 2 * k] == 0) {
				first = le(x + 32, 8);
				last = le(x + 40, 8);
			}
			for (k = 0; dname[k] && x[56 + 2 * k] == (uint8_t)dname[k] && x[57 + 2 * k] == 0; k++) {
			}
			if (dname[k] == 0 && x[56 + 2 * k] == 0 && x[57 + 2 * k] == 0) {
				dfirst = le(x + 32, 8);
				dlast = le(x + 40, 8);
			}
		}
		if (no == NULL && (first == 0 || last < first)) {
			no = "no xnusys in the GPT";
		} else if (no == NULL && (first != part_lba || last - first + 1 != part_blocks)) {
			no = "xnusys in the GPT is not where mtkloader said";
		}
		/* darwin: only with mtkufs_write=1, where mtkloader said, clear of xnusys and of the GPT, a window past its MiB */
		if (no == NULL && write_on) {
			const char *dno = NULL;

			if (dlba == 0 || dblocks == 0) {
				dno = "mtkloader gave no darwin";
			} else if (dfirst == 0 || dlast < dfirst) {
				dno = "no darwin in the GPT";
			} else if (dfirst != dlba || dlast - dfirst + 1 != dblocks) {
				dno = "darwin in the GPT is not where mtkloader said";
			} else if (!(dlast < first || dfirst > last) || dfirst <= GPT_ENTRIES + GPT_ENTRY_BLOCKS ||
			    dblocks <= DARWIN_KEEP || dlast > 0xffffffffULL) {
				dno = "darwin overlaps xnusys or the GPT, or is no bigger than its first MiB";
			}
			if (dno != NULL) {
				printf("mtkufs: darwin NOT opened for writing: %s (GPT: darwin %llu..%llu; mtkloader: %llu+%llu)\n", dno,
				    dfirst, dlast, dlba, dblocks);
			} else {
				darwin_ok = true;
				printf("mtkufs: GPT check: darwin %llu..%llu, as mtkloader said: open for writing past its first MiB "
				    "(mtkufs_write=1)\n", dfirst, dlast);
			}
		}
	}
	lck_mtx_unlock(&mufs_lock);
	if (no != NULL) {
		printf("mtkufs: GPT check FAILED: %s (GPT: xnusys %llu..%llu, header %u bytes, %u entries of %u at %llu; mtkloader: "
		    "%llu+%llu); not used\n", no, first, last, hsize, count, esize, entries, part_lba, part_blocks);
		return false;
	}
	printf("mtkufs: GPT check: header and entries CRCs good, xnusys %llu..%llu, as mtkloader said\n", first, last);
	return true;
}

/*
 * mtkufs_wtest=1, the dry run of writing: darwin blocks 256..511 (its second MiB) -- what they held (crc32), then a
 * pattern written eight times over, the cache synchronized, read back eight times and compared. Word i of darwin block
 * b holds b * 1024 + i (little-endian), so a dd of that MiB from Android can be checked against the same rule.
 */
static void
mufs_write_test(void)
{
	enum { BLOCKS = 256, ROUNDS = 8, CHUNK = BOUNCE / BLOCK };
	static uint32_t pattern[BLOCKS * BLOCK / 4];
	uint32_t crc_before = 0, crc_after = 0, bad = 0, bad_block = 0;
	uint64_t t, write_us = 0, sync_us = 0, read_us = 0, bytes = (uint64_t)ROUNDS * BLOCKS * BLOCK;
	int error = 0;

	for (uint32_t i = 0; i < BLOCKS * BLOCK / 4; i++) {
		pattern[i] = (DARWIN_KEEP + i / (BLOCK / 4)) * (BLOCK / 4) + i % (BLOCK / 4);
	}
	lck_mtx_lock(&mufs_lock);
	for (uint32_t b = 0; error == 0 && b < BLOCKS; b += CHUNK) {
		error = read_abs(dlba + DARWIN_KEEP + b, CHUNK, "darwin's second MiB, before");
		crc_before = crc32(crc_before, mufs_bounce, CHUNK * BLOCK);
	}
	t = mach_absolute_time();
	for (uint32_t r = 0; error == 0 && r < ROUNDS; r++) {
		for (uint32_t b = 0; error == 0 && b < BLOCKS; b += CHUNK) {
			error = write_abs(dlba + DARWIN_KEEP + b, CHUNK, (const uint8_t *)pattern + (size_t)b * BLOCK,
			    "darwin's second MiB, the pattern");
		}
	}
	write_us = us_since(t);
	t = mach_absolute_time();
	if (error == 0) {
		error = mufs_sync();
	}
	sync_us = us_since(t);
	t = mach_absolute_time();
	for (uint32_t r = 0; error == 0 && r < ROUNDS; r++) {
		crc_after = 0;
		for (uint32_t b = 0; error == 0 && b < BLOCKS; b += CHUNK) {
			error = read_abs(dlba + DARWIN_KEEP + b, CHUNK, "darwin's second MiB, read back");
			crc_after = crc32(crc_after, mufs_bounce, CHUNK * BLOCK);
			for (uint32_t k = 0; error == 0 && k < CHUNK; k++) {
				if (memcmp(mufs_bounce + (size_t)k * BLOCK, (const uint8_t *)pattern + (size_t)(b + k) * BLOCK, BLOCK) != 0) {
					if (bad++ == 0) {
						bad_block = DARWIN_KEEP + b + k;
					}
				}
			}
		}
	}
	read_us = us_since(t);
	lck_mtx_unlock(&mufs_lock);
	write_us += write_us == 0;
	read_us += read_us == 0;
	if (error != 0) {
		printf("mtkufs: write test STOPPED (%d): %llu writes done; see the line before\n", error, mufs_writes);
		return;
	}
	printf("mtkufs: write test: darwin blocks %u..%u (LUN 2 blocks %llu..%llu): crc32 before %08x; the pattern (crc32 "
	    "%08x) written %u times in %llu ms (%llu.%llu MB/s, %u requests of %u KiB), cache synchronized in %llu ms, read "
	    "back %u times in %llu ms (%llu.%llu MB/s): %s%s (crc32 %08x)\n", DARWIN_KEEP, DARWIN_KEEP + BLOCKS - 1,
	    dlba + DARWIN_KEEP, dlba + DARWIN_KEEP + BLOCKS - 1, crc_before, crc32(0, pattern, sizeof(pattern)), ROUNDS,
	    write_us / 1000, bytes / write_us, (bytes * 10 / write_us) % 10, ROUNDS * BLOCKS / CHUNK, CHUNK * BLOCK / 1024,
	    sync_us / 1000, ROUNDS, read_us / 1000, bytes / read_us, (bytes * 10 / read_us) % 10,
	    bad == 0 ? "the same every time" : "DIFFERENT", bad == 0 ? "" : " (blocks wrong, the first darwin block given)",
	    crc_after);
	if (bad != 0) {
		printf("mtkufs: write test: %u blocks read back wrong, the first darwin block %u\n", bad, bad_block);
	}
}

/* not used after all: LK's list back now, and nothing more sent */
static int
mufs_give_up(void)
{
	uint32_t left, cnr;
	boolean_t istate = ml_set_interrupts_enabled(FALSE);
	bool ok = mufs_put_back(&left, &cnr);

	ml_set_interrupts_enabled(istate);
	dead = true;
	printf("mtkufs: transfer list %s LK's %llx\n", ok ? "back to" : "NOT BACK TO", lk_utrl);
	return ENXIO;
}

static const char *
ahit_says(uint32_t ahit, char *s, size_t n)
{
	static const char *const scale[] = { "us", "0 us", "00 us", "ms", "0 ms", "00 ms", "?", "?" };

	if ((ahit & 0x3ff) == 0) {
		return "auto-hibernate off";
	}
	snprintf(s, n, "auto-hibernate after %u%s idle", ahit & 0x3ff, scale[(ahit >> 10) & 7]);
	return s;
}

/* ---- the start: once, from the root's hook or from hakios_input_init, whichever comes first ---- */

static int
mufs_start(void)
{
	uint32_t on = 0, cap, ahit, lastlba, blen;
	vm_offset_t cached = 0, bounce = 0;
	struct mufs_cmd c;
	uint8_t *cap8;
	char ahit_s[48];

	if (started) {
		return usable ? 0 : ENXIO;
	}
	started = true;
	if (!PE_parse_boot_argn("mtkufs", &on, sizeof(on)) || on == 0) {
		return ENXIO;
	}
	if (!PE_parse_boot_argn("mtkufs_lba", &part_lba, sizeof(part_lba)) ||
	    !PE_parse_boot_argn("mtkufs_blocks", &part_blocks, sizeof(part_blocks)) ||
	    !PE_parse_boot_argn("mtkufs_lkutrl", &lk_utrl, sizeof(lk_utrl))) {
		printf("mtkufs: mtkufs=1, but mtkloader gave no xnusys extent or LK's list (build it with XNU_UFS=1); the "
		    "controller is left alone\n");
		return ENXIO;
	}
	if (part_lba == 0 || part_blocks == 0 || part_lba + part_blocks > 0x100000000ULL || lk_utrl == 0 ||
	    (lk_utrl & 0x3ff)) {
		printf("mtkufs: mtkloader's xnusys %llu+%llu or LK's list %llx is not usable; the controller is left alone\n",
		    part_lba, part_blocks, lk_utrl);
		return ENXIO;
	}
	printf("mtkufs: xnusys at LUN 2 block %llu, %llu blocks of 4096 (from mtkloader)\n", part_lba, part_blocks);
	on = 0;
	write_on = PE_parse_boot_argn("mtkufs_write", &on, sizeof(on)) && on != 0;
	if (!PE_parse_boot_argn("var_store_crash_at", &crash_at, sizeof(crash_at))) {
		crash_at = 0;
	}
	PE_parse_boot_argn("mtkufs_data", &data_mode, sizeof(data_mode));
	PE_parse_boot_argn("mtkufs_crumbs", &crumbs_on, sizeof(crumbs_on));
	PE_parse_boot_argn("mtkufs_data_crash_at", &data_crash_at, sizeof(data_crash_at));
	on = 0;
	data_readback = PE_parse_boot_argn("mtkufs_readback", &on, sizeof(on)) && on != 0;
	if (write_on && (!PE_parse_boot_argn("mtkufs_dlba", &dlba, sizeof(dlba)) ||
	    !PE_parse_boot_argn("mtkufs_dblocks", &dblocks, sizeof(dblocks)))) {
		dlba = dblocks = 0;
	}
	if (!mufs_selftest()) {
		return ENXIO;
	}
	hc = ml_io_map(MUFS_PHYS, MUFS_SIZE);
	cap = rd(UFSHC_CAP);
	ahit = rd(UFSHC_AHIT);
	printf("mtkufs: as LK left it: CAP %08x, VER %08x, HCE %08x, HCS %08x, IS %08x, IE %08x, UTRIACR %08x\n", cap,
	    rd(UFSHC_VER), rd(UFSHC_HCE), rd(UFSHC_HCS), rd(UFSHC_IS), rd(UFSHC_IE), rd(UFSHC_UTRIACR));
	printf("mtkufs: as LK left it: AHIT %08x (%s)\n", ahit, ahit_says(ahit, ahit_s, sizeof(ahit_s)));
	printf("mtkufs: as LK left it: transfer list %08x:%08x, doorbell %08x, run %08x, completion %08x; task management "
	    "list %08x, doorbell %08x, run %08x\n", rd(UFSHC_UTRLBAU), rd(UFSHC_UTRLBA), rd(UFSHC_UTRLDBR),
	    rd(UFSHC_UTRLRSR), rd(UFSHC_UTRLCNR), rd(UFSHC_UTMRLBA), rd(UFSHC_UTMRLDBR), rd(UFSHC_UTMRLRSR));
	if ((cap & 0x1f) + 1 < SLOTS || (cap & (1u << 24)) == 0) {
		printf("mtkufs: not used: fewer than 32 slots or no 64-bit addressing\n");
		return ENXIO;
	}

	/* the list and command descriptors: one piece, its cached view written out and not used again, mapped uncached */
	if (kmem_alloc_contig(kernel_map, &cached, round_page(AREA_SIZE), PAGE_MASK, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA,
	    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS ||
	    kmem_alloc_contig(kernel_map, &bounce, BOUNCE, PAGE_MASK, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA,
	    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		printf("mtkufs: not used: no memory in one piece\n");
		return ENXIO;
	}
	mufs_area_pa = kvtophys(cached);
	FlushPoC_DcacheRegion(cached, round_page(AREA_SIZE));
	mufs_list = (volatile uint32_t *)ml_io_map_unmappable((vm_offset_t)mufs_area_pa, round_page(AREA_SIZE), VM_WIMG_WCOMB);
	mufs_ucd = (vm_offset_t)mufs_list + AREA_UCD;
	mufs_bounce = (uint8_t *)bounce;
	mufs_bounce_pa = kvtophys(bounce);
	FlushPoC_DcacheRegion(bounce, BOUNCE);
	for (uint32_t i = 0; i < SLOTS; i++) {
		make_tur(&c, (uint8_t)i);
		if (mufs_place(i, &c, 0) != 0) {
			printf("mtkufs: not used: slot %u would not take its TEST UNIT READY\n", i);
			return ENXIO;
		}
	}
	if (mufs_switch_in() != 0) {
		return ENXIO;
	}

	/* the unit's size, and xnusys inside it */
	make_capacity(&c);
	FlushPoC_DcacheRegion(bounce, PAGE_SIZE);
	if (mufs_place(SLOT_READ, &c, 8) != 0 || mufs_ring(SLOT_READ, "READ CAPACITY(10)") != 0) {
		return mufs_give_up();
	}
	FlushPoC_DcacheRegion(bounce, PAGE_SIZE);
	cap8 = mufs_bounce;
	lastlba = be32(cap8);
	blen = be32(cap8 + 4);
	printf("mtkufs: READ CAPACITY(10): last block %u, %u bytes a block\n", lastlba, blen);
	if (blen != BLOCK || part_lba + part_blocks - 1 > lastlba) {
		printf("mtkufs: not used: xnusys does not fit the unit as it says it is\n");
		return mufs_give_up();
	}
	if (!mufs_gpt_check()) {
		return mufs_give_up();
	}
	usable = true;
	if (darwin_ok && (dlba + dblocks - 1 > lastlba || !mufs_selftest_darwin())) {
		darwin_ok = false;
		printf("mtkufs: darwin closed again: past the unit's end, or the whitelist for it wrong\n");
	}
	on = 0;
	if (darwin_ok && PE_parse_boot_argn("mtkufs_wtest", &on, sizeof(on)) && on != 0) {
		mufs_write_test();
	}

	/* the device as long as xnusys's APFS containers, back to back, as the Hot 20's eMMC driver has it (md0's size) */
	dev_blocks = part_blocks;
	for (uint64_t at = 0; at < part_blocks * BLOCK;) {
		static uint32_t sb[BLOCK / 4];
		uint64_t n;
		uint32_t bs;

		if (mufs_read(at, (uint8_t *)sb, BLOCK) != 0 || sb[8] != 0x4253584eu) {     /* "NXSB" */
			if (at != 0) {
				dev_blocks = at / BLOCK;
			}
			break;
		}
		bs = sb[9];
		memcpy(&n, (uint8_t *)sb + 40, 8);
		if (bs < BLOCK || bs > 65536 || bs % BLOCK || n == 0 || n > (part_blocks * BLOCK - at) / bs) {
			dev_blocks = at != 0 ? at / BLOCK : part_blocks;
			break;
		}
		at += n * bs;
		dev_blocks = at / BLOCK;
		apfs_found = true;
	}
	printf("mtkufs: /dev/mtkufs0 is %llu MB of xnusys's %llu MB (%s)\n", (dev_blocks * BLOCK) >> 20,
	    (part_blocks * BLOCK) >> 20, apfs_found ? "its APFS containers" : "no APFS container: all of it");

	/* the first blocks of xnusys, to set beside a dd of it from Android */
	for (uint32_t b = 0; b < 4; b++) {
		static uint8_t block[BLOCK];
		const uint8_t *p = block;

		if (mufs_read((uint64_t)b * BLOCK, block, BLOCK) != 0) {
			break;
		}
		printf("mtkufs: xnusys block %u: crc32 %08x, %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x "
		    "%08x %08x\n", b, crc32(0, block, BLOCK),
		    be32(p), be32(p + 4), be32(p + 8), be32(p + 12), be32(p + 16), be32(p + 20), be32(p + 24), be32(p + 28),
		    be32(p + 32), be32(p + 36), be32(p + 40), be32(p + 44), be32(p + 48), be32(p + 52), be32(p + 56), be32(p + 60));
	}
	return 0;
}

/* ---- the data volume's store: the POCO's handshake and claim (drivers/storage/hakios.ufs.c), for var_store.c ---- */

#define VAR_SAVE_NONE      0
#define VAR_SAVE_IDLE      1
#define VAR_SAVE_WANTED    2
#define VAR_SAVE_DONE      3
#define VAR_SAVE_MS        20000        /* at most this long on the way out (var_save_ms=N) */

static volatile int var_save = VAR_SAVE_NONE;
static int var_busy_pid;
static uint64_t var_store_at = DARWIN_KEEP * (uint64_t)BLOCK;

SYSCTL_DECL(_kern_hakios);
SYSCTL_QUAD(_kern_hakios, OID_AUTO, var_store_at, CTLFLAG_RD | CTLFLAG_LOCKED, &var_store_at,
    "where in darwin the data volume's store starts, in bytes (past the boot settings block)");

/* 1 registers the saver, 3 says the volume is in the flash; 2 is the kernel's */
static int
sysctl_var_save(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	int value = var_save, error = sysctl_io_number(req, value, sizeof(value), &value, NULL);

	if (error != 0 || req->newptr == 0) {
		return error;
	}
	if (value == VAR_SAVE_IDLE && var_save != VAR_SAVE_WANTED) {
		var_save = VAR_SAVE_IDLE;
	} else if (value == VAR_SAVE_DONE && var_save == VAR_SAVE_WANTED) {
		var_save = VAR_SAVE_DONE;
	} else if (value == VAR_SAVE_NONE) {
		var_save = VAR_SAVE_NONE;
	} else {
		return EINVAL;
	}
	return 0;
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, var_save, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED, NULL, 0, sysctl_var_save, "I",
    "the data volume's saver: 1 registers, 2 is the kernel asking, 3 is done");

/* 1 takes the claim (from a holder that is gone, too), 0 gives it back; reading says who holds it */
static int
sysctl_var_busy(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	int value = var_busy_pid, me = proc_selfpid(), error = sysctl_io_number(req, value, sizeof(value), &value, NULL);

	if (error != 0 || req->newptr == 0) {
		return error;
	}
	lck_mtx_lock(&mufs_lock);
	if (value == 1) {
		if (var_busy_pid != 0 && var_busy_pid != me) {
			proc_t held = proc_find(var_busy_pid);

			if (held != NULL) {
				proc_rele(held);
				error = EBUSY;
			} else {
				printf("var_store: the claim on the store was left by process %d, which is gone\n", var_busy_pid);
			}
		}
		if (error == 0) {
			var_busy_pid = me;
		}
	} else if (value == 0) {
		if (var_busy_pid == me || var_busy_pid == 0) {
			var_busy_pid = 0;
		} else {
			error = EPERM;
		}
	} else {
		error = EINVAL;
	}
	lck_mtx_unlock(&mufs_lock);
	return error;
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, var_busy, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED, NULL, 0, sysctl_var_busy, "I",
    "the claim a writer of /dev/rufs_darwin must hold: 1 takes it, 0 gives it back");

/*
 * /dev/rufs_darwin: bytes of darwin from its start, readable and writable past its first MiB only. Writes in whole
 * blocks, by the claim's holder, each piece read back and compared before the next.
 */
static int
darwin_rw(struct uio *uio, bool writing)
{
	uint64_t window = DARWIN_KEEP * (uint64_t)BLOCK, end = dblocks * (uint64_t)BLOCK;
	int error = 0;

	if (!usable || dead || !darwin_ok) {
		return ENXIO;
	}
	if (writing && (var_busy_pid == 0 || var_busy_pid != proc_selfpid())) {
		mufs_refused++;
		printf("mtkufs: a write to /dev/rufs_darwin by process %d without the claim (kern.hakios.var_busy): refused\n",
		    proc_selfpid());
		return EPERM;
	}
	while (error == 0 && uio_resid(uio) > 0) {
		uint64_t off = (uint64_t)uio_offset(uio), len = (uint64_t)uio_resid(uio), first, skip, blocks, take;

		if (off == end && !writing) {
			break;                          /* the end: nothing more to read */
		}
		if (off < window || off >= end) {
			return writing ? EPERM : EINVAL;
		}
		if (len > end - off) {
			len = end - off;
		}
		if (writing && (off % BLOCK != 0 || len % BLOCK != 0)) {
			return EINVAL;
		}
		first = off / BLOCK;
		skip = off % BLOCK;
		blocks = (skip + len + BLOCK - 1) / BLOCK;
		if (blocks > BOUNCE / BLOCK) {
			blocks = BOUNCE / BLOCK;
		}
		take = blocks * BLOCK - skip;
		if (take > len) {
			take = len;
		}
		lck_mtx_lock(&mufs_lock);
		if (writing) {
			error = uiomove((caddr_t)darwin_buf, (int)take, uio);
			if (error == 0) {
				darwin_dirty = true;
				error = write_abs(dlba + first, (uint32_t)blocks, darwin_buf, "/dev/rufs_darwin");
			}
			if (error == 0 && ++darwin_writes == crash_at) {
				register uint64_t x0 __asm__("x0") = 0x84000009;        /* PSCI SYSTEM_RESET */

				printf("mtkufs: var_store_crash_at=%u: write %u of /dev/rufs_darwin done (darwin blocks %llu..%llu): "
				    "the phone is reset now\n", crash_at, darwin_writes, first, first + blocks - 1);
				if (psci_smp_hvc) {
					__asm__ volatile ("hvc #0" : "+r" (x0) : : "x1", "x2", "x3", "memory");
				} else {
					__asm__ volatile ("smc #0" : "+r" (x0) : : "x1", "x2", "x3", "memory");
				}
			}
			if (error == 0) {
				error = read_abs(dlba + first, (uint32_t)blocks, "/dev/rufs_darwin, read back");
			}
			if (error == 0 && memcmp(mufs_bounce, darwin_buf, (size_t)take) != 0) {
				printf("mtkufs: darwin blocks %llu..%llu read back differently from what was written\n", first,
				    first + blocks - 1);
				error = EIO;
			}
		} else {
			error = read_abs(dlba + first, (uint32_t)blocks, "/dev/rufs_darwin");
			if (error == 0) {
				mufs_reads++;
				memcpy(darwin_buf, mufs_bounce + skip, (size_t)take);
				error = uiomove((caddr_t)darwin_buf, (int)take, uio);
			}
		}
		if (error != 0) {
			mufs_errors++;
		}
		lck_mtx_unlock(&mufs_lock);
	}
	return error;
}

extern uint32_t loader_log_copy(char *dst, uint32_t max);       /* osfmk/arm64/loader_return.c */
#define CONSOLE_MAX         (2816u << 10)       /* loader_return's head and ring: the console it keeps */

static int
write_var_file(const char *path, const char *data, uint32_t len)
{
	vfs_context_t ctx = vfs_context_kernel();
	vnode_t vp = NULLVP;
	int error = vnode_open(path, O_CREAT | O_TRUNC | FWRITE, 0644, 0, &vp, ctx);

	if (error == 0) {
		error = vn_rdwr(UIO_WRITE, vp, (caddr_t)(uintptr_t)data, (int)len, 0, UIO_SYSSPACE, IO_NOAUTH,
		    vfs_context_ucred(ctx), NULL, vfs_context_proc(ctx));
		vnode_close(vp, FWRITE, ctx);
	}
	printf("var_store: %u bytes into %s: %s (%d)\n", len, path, error == 0 ? "written" : "NOT written", error);
	return error;
}

/*
 * var_store_log=1: the kernel's message buffer, oldest first, into /private/var/root/hakios-run.log, and the console as
 * loader_return keeps it (its first 256 KB and last 2.5 MB: the processes' lines, tools/latency.py's "tl:") into
 * /private/var/root/hakios-console.log, so that the store
 * saved next keeps it -- the log of a run that ends in a reset, which no memory survives (read back from Android with
 * tools/var-store-read.py extract root/hakios-run.log; not under log/, whose contents the store leaves out).
 */
static void
log_into_var(void)
{
	uint32_t on = 0, size, x, from = 0;
	char *copy;

	if (!PE_parse_boot_argn("var_store_log", &on, sizeof(on)) || on == 0 || msgbufp == NULL ||
	    msgbufp->msg_magic != MSG_MAGIC || msgbufp->msg_size <= 0) {
		return;
	}
	size = (uint32_t)msgbufp->msg_size;
	copy = kalloc_data(size, Z_WAITOK);
	if (copy == NULL) {
		return;
	}
	bsd_log_lock_safe();
	x = (uint32_t)msgbufp->msg_bufx % size;
	memcpy(copy, msgbufp->msg_bufc + x, size - x);
	memcpy(copy + size - x, msgbufp->msg_bufc, x);
	bsd_log_unlock();
	while (from < size && copy[from] == 0) {
		from++;                                 /* a buffer not yet gone round: its unused start */
	}
	(void)write_var_file("/private/var/root/hakios-run.log", copy + from, size - from);
	kfree_data(copy, size);
	copy = kalloc_data(CONSOLE_MAX, Z_WAITOK);
	if (copy != NULL) {
		(void)write_var_file("/private/var/root/hakios-console.log", copy, loader_log_copy(copy, CONSOLE_MAX));
		kfree_data(copy, CONSOLE_MAX);
	}
}

/*
 * The orderly end of a run (loader_return.c, and the restart hook): if /dev/mtkufs_data was written, the file systems
 * synced and its cache synchronized; then ask the saver for the data volume and wait, bounded, while the system is
 * still whole. No saver registered: no wait.
 */
void
hakios_var_save_on_return(void)
{
	uint32_t ms = VAR_SAVE_MS;
	uint64_t waited = 0;

	if (data_written && !dead) {
		int error;

		(void)sync_internal();
		error = data_sync();
		printf("mtkufs: the run is ending: file systems synced, the data volume's cache synchronized (%d); %u writes "
		    "to it this boot\n", error, data_writes);
	}
	if (!usable || !darwin_ok || var_save != VAR_SAVE_IDLE) {
		return;
	}
	PE_parse_boot_argn("var_save_ms", &ms, sizeof(ms));
	log_into_var();
	printf("var_store: the run is ending: the data volume is asked for\n");
	var_save = VAR_SAVE_WANTED;
	while (var_save != VAR_SAVE_DONE && waited < ms) {
		IOSleep(20);
		waited += 20;
	}
	if (var_save == VAR_SAVE_DONE) {
		printf("var_store: the saver has finished after %llu ms\n", waited);
	} else {
		printf("var_store: the saver did not answer in %u ms: the store keeps what it had\n", ms);
	}
}

/*
 * Breadcrumbs (mtkufs_crumbs=1, with mtkufs_write=1): a line of the kernel's, one block each, straight onto darwin's MiB
 * after the store (CRUMB_FIRST), the device's cache synchronized after each, so that what a run said before a reset
 * that no memory survives is still there: "crumb <this boot's counter at its first> <n> <seconds>: <line>". Read from
 * Android: dd if=/dev/block/by-name/darwin bs=4096 skip=1048832 count=256 | strings. The first CRUMB_BLOCKS of a boot
 * only; each boot starts again at the first block.
 */
int
hakios_mt6781_ufs_crumb(const char *text)
{
	uint64_t ns;
	int error, n;

	if (!crumbs_on || !usable || dead || !write_on || !darwin_ok || dblocks <= DATA_FIRST || crumb_next >= CRUMB_BLOCKS) {
		return ENXIO;
	}
	absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
	lck_mtx_lock(&mufs_lock);
	if (crumb_boot == 0) {
		crumb_boot = __builtin_arm_rsr64("CNTPCT_EL0");
	}
	memset(darwin_buf, 0, BLOCK);
	n = snprintf((char *)darwin_buf, BLOCK, "crumb %llx %u %llu.%03llu: %s\n", crumb_boot, crumb_next,
	    ns / NSEC_PER_SEC, ns % NSEC_PER_SEC / NSEC_PER_MSEC, text);
	(void)n;
	error = write_abs(dlba + CRUMB_FIRST + crumb_next, 1, darwin_buf, "a breadcrumb");
	if (error == 0) {
		error = mufs_sync();
	}
	if (error == 0) {
		crumb_next++;
	} else {
		mufs_errors++;
	}
	lck_mtx_unlock(&mufs_lock);
	return error;
}

/* ---- the data volume: darwin from DATA_FIRST to its end ---- */

static uint64_t
data_bytes(void)
{
	return data_mode != 0 && usable && darwin_ok && dblocks > DATA_FIRST ? (dblocks - DATA_FIRST) * BLOCK : 0;
}

static void
data_crash(uint64_t first, uint64_t blocks)
{
	register uint64_t x0 __asm__("x0") = 0x84000009;        /* PSCI SYSTEM_RESET */

	printf("mtkufs: mtkufs_data_crash_at=%u: write %u of /dev/mtkufs_data done (its blocks %llu..%llu): the phone is "
	    "reset now\n", data_crash_at, data_writes, first, first + blocks - 1);
	if (psci_smp_hvc) {
		__asm__ volatile ("hvc #0" : "+r" (x0) : : "x1", "x2", "x3", "memory");
	} else {
		__asm__ volatile ("smc #0" : "+r" (x0) : : "x1", "x2", "x3", "memory");
	}
}

/*
 * Bytes [offset, offset + len) of the data volume, read into p or written from it, a bounce buffer's worth at a time;
 * a write that is not whole blocks of 4096 has the blocks around it read and patched first.
 */
static int
data_io(bool writing, uint64_t offset, uint8_t *p, uint64_t len)
{
	uint64_t size = data_bytes();
	int error = 0;

	if (size == 0 || dead) {
		return ENXIO;
	}
	if (offset > size || len > size - offset) {
		return EINVAL;
	}
	if (writing && data_mode < 2) {
		return EROFS;
	}
	lck_mtx_lock(&mufs_lock);
	while (len > 0 && error == 0) {
		uint64_t first = offset / BLOCK, skip = offset % BLOCK, blocks = (skip + len + BLOCK - 1) / BLOCK, take;

		if (blocks > BOUNCE / BLOCK) {
			blocks = BOUNCE / BLOCK;
		}
		take = MIN(blocks * BLOCK - skip, len);
		if (!writing) {
			error = read_abs(dlba + DATA_FIRST + first, (uint32_t)blocks, "/dev/mtkufs_data");
			if (error == 0) {
				memcpy(p, mufs_bounce + skip, (size_t)take);
				mufs_reads++;
			}
		} else {
			if (skip != 0 || take != blocks * BLOCK) {
				error = read_abs(dlba + DATA_FIRST + first, (uint32_t)blocks, "/dev/mtkufs_data, around a part");
				if (error == 0) {
					memcpy(darwin_buf, mufs_bounce, (size_t)blocks * BLOCK);
				}
			}
			if (error == 0) {
				memcpy(darwin_buf + skip, p, (size_t)take);
				darwin_dirty = data_written = true;
				error = write_abs(dlba + DATA_FIRST + first, (uint32_t)blocks, darwin_buf, "/dev/mtkufs_data");
			}
			if (error == 0 && ++data_writes == data_crash_at) {
				data_crash(first, blocks);
			}
			if (error == 0 && data_readback) {
				error = read_abs(dlba + DATA_FIRST + first, (uint32_t)blocks, "/dev/mtkufs_data, read back");
				if (error == 0 && memcmp(mufs_bounce, darwin_buf, (size_t)blocks * BLOCK) != 0) {
					printf("mtkufs: data volume blocks %llu..%llu read back differently from what was written\n",
					    first, first + blocks - 1);
					error = EIO;
				}
			}
		}
		if (error != 0) {
			mufs_errors++;
		}
		offset += take;
		p += take;
		len -= take;
	}
	lck_mtx_unlock(&mufs_lock);
	return error;
}

static void
data_strategy(buf_t bp)
{
	uint64_t offset = (uint64_t)buf_blkno(bp) * data_bsize, len = buf_count(bp), size = data_bytes();
	caddr_t mapped;
	int error;

	if (offset >= size) {
		buf_seterror(bp, offset == size ? 0 : EINVAL);
		buf_setresid(bp, buf_count(bp));
		buf_biodone(bp);
		return;
	}
	len = MIN(len, size - offset);
	/* HFS Plus's cluster I/O hands over pages with no kernel address yet: buf_map() gives them one */
	if ((error = buf_map(bp, &mapped)) == 0) {
		error = data_io(!(buf_flags(bp) & B_READ), offset, (uint8_t *)mapped, len);
		buf_unmap(bp);
	}
	buf_setresid(bp, (uint32_t)(buf_count(bp) - (error ? 0 : len)));
	if (error) {
		buf_seterror(bp, error);
	}
	buf_biodone(bp);
}

static int
data_sync(void)
{
	int error = 0;

	lck_mtx_lock(&mufs_lock);
	if (darwin_dirty && !dead) {
		error = mufs_sync();
		if (error == 0) {
			darwin_dirty = false;
		}
	}
	lck_mtx_unlock(&mufs_lock);
	return error;
}

static int
data_ioctl(u_long cmd, caddr_t data)
{
	switch (cmd) {
	case DKIOCGETBLOCKSIZE:
		*(uint32_t *)data = data_bsize;
		return 0;
	case DKIOCSETBLOCKSIZE:
		if (*(uint32_t *)data < SECTOR || *(uint32_t *)data > BLOCK || (*(uint32_t *)data & (*(uint32_t *)data - 1)) != 0) {
			return EINVAL;
		}
		data_bsize = *(uint32_t *)data;
		return 0;
	case DKIOCGETPHYSICALBLOCKSIZE:
		*(uint32_t *)data = BLOCK;
		return 0;
	case DKIOCGETBLOCKCOUNT:
		*(uint64_t *)data = data_bytes() / data_bsize;
		return 0;
	case DKIOCISWRITABLE:
		*(uint32_t *)data = data_mode >= 2;
		return 0;
	case DKIOCGETMAXBLOCKCOUNTREAD:
	case DKIOCGETMAXBLOCKCOUNTWRITE:
		*(uint64_t *)data = BOUNCE / data_bsize;
		return 0;
	case DKIOCSYNCHRONIZE:
	case DKIOCSYNCHRONIZECACHE:
		return data_sync();
	default:
		return ENOTTY;
	}
}

/* ---- the devices ---- */

static int
mufs_open(dev_t dev, int flags, __unused int devtype, __unused proc_t p)
{
	if (minor(dev) == DARWIN_MINOR) {
		return usable && darwin_ok && !dead ? 0 : ENXIO;
	}
	if (minor(dev) == DATA_MINOR) {
		return data_bytes() == 0 || dead ? ENXIO : (flags & FWRITE) && data_mode < 2 ? EROFS : 0;
	}
	return (flags & FWRITE) ? EROFS : 0;
}

static int
mufs_close(dev_t dev, __unused int flags, __unused int devtype, __unused proc_t p)
{
	int error = 0;

	if (minor(dev) == DATA_MINOR) {
		return data_sync();
	}
	if (minor(dev) == DARWIN_MINOR && darwin_dirty) {
		lck_mtx_lock(&mufs_lock);
		error = mufs_sync();
		if (error == 0) {
			darwin_dirty = false;
		}
		lck_mtx_unlock(&mufs_lock);
	}
	return error;
}

static void
mufs_strategy(buf_t bp)
{
	uint64_t sectors = dev_blocks * (BLOCK / SECTOR), at = (uint64_t)buf_blkno(bp);
	uint32_t count = buf_count(bp);
	int error = 0;

	if (minor(buf_device(bp)) == DATA_MINOR) {
		data_strategy(bp);
		return;
	}
	if (!(buf_flags(bp) & B_READ)) {
		error = EROFS;                          /* this device never writes */
	} else if (at >= sectors) {
		buf_setresid(bp, count);
		error = at == sectors ? 0 : EINVAL;
	} else {
		if (count > (sectors - at) * SECTOR) {
			count = (uint32_t)((sectors - at) * SECTOR);
		}
		error = mufs_read(at * SECTOR, (uint8_t *)buf_dataptr(bp), count);
		buf_setresid(bp, error == 0 ? buf_count(bp) - count : buf_count(bp));
	}
	if (error) {
		buf_seterror(bp, error);
	}
	buf_biodone(bp);
}

static int
mufs_ioctl(dev_t dev, u_long cmd, caddr_t data, __unused int fflag, __unused proc_t p)
{
	if (minor(dev) == DATA_MINOR) {
		return data_ioctl(cmd, data);
	}
	switch (cmd) {
	case DKIOCGETBLOCKSIZE:
		*(uint32_t *)data = SECTOR;
		return 0;
	case DKIOCSETBLOCKSIZE:
		return *(uint32_t *)data == SECTOR ? 0 : EINVAL;
	case DKIOCGETBLOCKCOUNT:
		*(uint64_t *)data = dev_blocks * (BLOCK / SECTOR);
		return 0;
	case DKIOCISWRITABLE:
		*(uint32_t *)data = 0;
		return 0;
	case DKIOCGETMAXBLOCKCOUNTREAD:
		*(uint64_t *)data = BOUNCE / SECTOR;
		return 0;
	default:
		return ENOTTY;
	}
}

static int
mufs_size(dev_t dev)
{
	return minor(dev) == DATA_MINOR ? (int)data_bsize : SECTOR;
}

static int
mufs_rawread(dev_t dev, struct uio *uio, __unused int ioflag)
{
	if (minor(dev) == DARWIN_MINOR) {
		return darwin_rw(uio, false);
	}
	return physio(mufs_strategy, NULL, dev, B_READ, (u_int (*)(buf_t))mufs_size, uio, SECTOR);
}

static int
mufs_rawwrite(dev_t dev, struct uio *uio, __unused int ioflag)
{
	return minor(dev) == DARWIN_MINOR ? darwin_rw(uio, true) : EROFS;
}

static const struct bdevsw mufs_bdevsw = {
	.d_open     = mufs_open,
	.d_close    = mufs_close,
	.d_strategy = mufs_strategy,
	.d_ioctl    = mufs_ioctl,
	.d_dump     = eno_dump,
	.d_psize    = mufs_size,
	.d_type     = D_DISK,
};

static const struct cdevsw mufs_cdevsw = {
	.d_open       = mufs_open,
	.d_close      = mufs_close,
	.d_read       = mufs_rawread,
	.d_write      = mufs_rawwrite,
	.d_ioctl      = mufs_ioctl,
	.d_stop       = eno_stop,
	.d_reset      = eno_reset,
	.d_ttys       = NULL,
	.d_select     = eno_select,
	.d_mmap       = eno_mmap,
	.d_strategy   = eno_strat,
	.d_reserved_1 = eno_getc,
	.d_reserved_2 = eno_putc,
	.d_type       = D_DISK,
};

static int
mufs_devsw(void)
{
	if (bmajor >= 0) {
		return 0;
	}
	bmajor = bdevsw_add(-1, &mufs_bdevsw);
	if (bmajor < 0 || (cmajor = cdevsw_add_with_bdev(-1, &mufs_cdevsw, bmajor)) < 0) {
		printf("mtkufs: no room among the devices (%d, %d)\n", bmajor, cmajor);
		bmajor = -1;
		return ENXIO;
	}
	return 0;
}

/* mtkufs_root=1 (IOFindBSDRoot): xnusys as the root */
int
hakios_mtk_ufs_root(dev_t *root)
{
	if (mufs_start() != 0) {
		return ENXIO;
	}
	if (!apfs_found) {
		printf("mtkufs: no APFS container at the start of xnusys (the system image is written there): not the root\n");
		return ENXIO;
	}
	if (mufs_devsw() != 0) {
		return ENXIO;
	}
	*root = makedev(bmajor, 0);
	return 0;
}

/* from hakios_input_init, devfs up: started if the root did not start it, and the nodes */
void
hakios_mt6781_ufs_init(void)
{
	if (mufs_start() != 0 || mufs_devsw() != 0) {
		return;
	}
	if (devfs_make_node(makedev(bmajor, 0), DEVFS_BLOCK, UID_ROOT, GID_OPERATOR, 0640, "mtkufs0") == NULL ||
	    devfs_make_node(makedev(cmajor, 0), DEVFS_CHAR, UID_ROOT, GID_OPERATOR, 0640, "rmtkufs0") == NULL) {
		printf("mtkufs: no /dev nodes\n");
	}
	if (darwin_ok && devfs_make_node(makedev(cmajor, DARWIN_MINOR), DEVFS_CHAR, UID_ROOT, GID_WHEEL, 0600,
	    "rufs_darwin") == NULL) {
		printf("mtkufs: no /dev/rufs_darwin\n");
	}
	if (data_bytes() != 0) {
		if (devfs_make_node(makedev(bmajor, DATA_MINOR), DEVFS_BLOCK, UID_ROOT, GID_OPERATOR, 0640, "mtkufs_data") ==
		    NULL) {
			printf("mtkufs: no /dev/mtkufs_data\n");
		} else {
			printf("mtkufs: /dev/mtkufs_data: darwin blocks %u..%llu, %llu blocks of 4096 (%llu MiB), %s%s\n",
			    DATA_FIRST, dblocks - 1, dblocks - DATA_FIRST, (dblocks - DATA_FIRST) / 256,
			    data_mode >= 2 ? "writable" : "read only (mtkufs_data=1)",
			    data_mode >= 2 && data_readback ? ", every write read back" : "");
			/* the volume headers' signatures (HFS Plus: "H+" or "HX" at 1024 and at its end - 1024), read once */
			uint8_t head[8], tail[8];

			if (data_io(false, 1024, head, 8) == 0 && data_io(false, data_bytes() - 1024, tail, 8) == 0) {
				printf("mtkufs: /dev/mtkufs_data: at 1024 %02x%02x%02x%02x%02x%02x%02x%02x, at its end - 1024 "
				    "%02x%02x%02x%02x%02x%02x%02x%02x\n", head[0], head[1], head[2], head[3], head[4], head[5],
				    head[6], head[7], tail[0], tail[1], tail[2], tail[3], tail[4], tail[5], tail[6], tail[7]);
			}
		}
	} else if (data_mode != 0) {
		printf("mtkufs: mtkufs_data=%u, but no darwin past the store's slots (mtkufs_write=1 needed): no "
		    "/dev/mtkufs_data\n", data_mode);
	}
}

/*
 * Before the kernel gives the phone back to mtkloader (loader_return, the other processors stopped): LK's list
 * back in. No locks (a stopped processor may hold one), no printf.
 */
void
hakios_mt6781_ufs_quiesce(void)
{
	uint32_t left = 0, cnr = 0;
	bool ok;

	if (!in) {
		if (started && usable) {
			say_late("mtkufs: at the end: LK's list already in; %llu reads, %llu writes, %llu errors, %llu refused\n",
			    mufs_reads, mufs_writes, mufs_errors, mufs_refused);
		}
		return;
	}
	if (darwin_dirty && !dead) {
		(void)mufs_sync();              /* what was written out of the device's cache before LK has it */
	}
	ok = mufs_put_back(&left, &cnr);
	say_late("mtkufs: at the end: transfer list %s LK's %llx (doorbell on entry %08x, completion bits of this kernel's "
	    "cleared %08x); %llu reads, %llu writes, %llu errors, %llu refused\n", ok ? "back to" : "NOT BACK TO", lk_utrl,
	    left, cnr, mufs_reads, mufs_writes, mufs_errors, mufs_refused);
}
