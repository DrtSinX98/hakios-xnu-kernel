#!/usr/bin/env python3
"""
Take space off the end of a partition and put new ones in its place, in a GPT read from the phone:

  gpt-carve.py <gpt-primary.bin> <gpt-secondary.bin> <total-sectors> [--sector N] [NAME=GIB[:TYPE] ...]

Writes the new primary and secondary images beside the originals (.new), reads them back and checks them as a
firmware would; nothing touches the phone. <gpt-primary.bin> is the first sectors of the unit (protective MBR,
header, entries), <gpt-secondary.bin> its last ones (entries, then the backup header in the very last sector);
<total-sectors> is the unit's size in sectors of --sector bytes.

The X6826B (eMMC, 512-byte sectors): 4 GiB off the end of userdata becomes "xnusys", where userland/system.packfs
lives so that mtkloader can read it straight off the flash (the default). The realme 8i (UFS, 4096-byte sectors,
userdata on LU 0 followed by flashinfo): --sector 4096 and the partitions it wants, in the order they are to lie
after userdata. Everything else -- every other entry, the disk GUID, the entry order -- is left exactly as it was.
"""
import argparse
import binascii
import struct
import uuid

SHRINK = "userdata"
TYPES = {
    "linux": uuid.UUID("0fc63daf-8483-4772-8e79-3d69d8477de4"),     # Linux filesystem data (the X6826B's xnusys)
    "apfs": uuid.UUID("7c3457ef-0000-11aa-aa11-00306543ecac"),      # APFS container (tools/userdata-carve.py's darwin)
}
ALIGN_BYTES = 1 << 20                   # 1 MiB


def crc32(b):
    return binascii.crc32(b) & 0xffffffff


def name_of(entry):
    return entry[56:128].decode("utf-16-le").rstrip("\0")


def parse(hdr):
    sig, rev, hsize, hcrc, _, mylba, altlba, first, last = struct.unpack_from("<8sIIIIQQQQ", hdr, 0)
    guid = hdr[56:72]
    entlba, nent, entsize, entcrc = struct.unpack_from("<QIII", hdr, 72)
    assert sig == b"EFI PART", "not a GPT header"
    return dict(rev=rev, hsize=hsize, mylba=mylba, altlba=altlba, first=first, last=last,
                guid=guid, entlba=entlba, nent=nent, entsize=entsize, hcrc=hcrc, entcrc=entcrc)


def build_header(h, entries, mylba, altlba, entlba, sector):
    out = bytearray(h["hsize"])
    struct.pack_into("<8sIIII", out, 0, b"EFI PART", h["rev"], h["hsize"], 0, 0)
    struct.pack_into("<QQQQ", out, 24, mylba, altlba, h["first"], h["last"])
    out[56:72] = h["guid"]
    struct.pack_into("<QIII", out, 72, entlba, h["nent"], h["entsize"], crc32(entries))
    struct.pack_into("<I", out, 16, crc32(bytes(out)))
    return bytes(out) + b"\0" * (sector - h["hsize"])


def table(hdr_bytes, image, ent_off, sector, label):
    """Check one copy as a firmware would (header CRC, entry CRC) and return its partitions."""
    h = parse(hdr_bytes)
    raw = bytearray(hdr_bytes[:h["hsize"]])
    struct.pack_into("<I", raw, 16, 0)
    assert crc32(bytes(raw)) == h["hcrc"], f"{label}: the header does not match its checksum"
    ents = image[ent_off:ent_off + h["nent"] * h["entsize"]]
    assert crc32(bytes(ents)) == h["entcrc"], f"{label}: the entry array does not match its checksum"
    parts = []
    for i in range(h["nent"]):
        e = ents[i * h["entsize"]:(i + 1) * h["entsize"]]
        if e[0:16] != b"\0" * 16:
            s, t = struct.unpack_from("<QQ", e, 32)
            parts.append((s, t, name_of(e), i))
    return h, sorted(parts)


def show(parts, sector, title, around):
    print(title)
    for s, t, nm, i in parts:
        if nm in around:
            print(f"  {nm:12s} entry {i:3d}  {s:>10d}..{t:<10d}  {(t - s + 1) * sector / 2**30:8.2f} GiB")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("primary")
    ap.add_argument("secondary")
    ap.add_argument("total", type=int, help="the unit's size in sectors")
    ap.add_argument("--sector", type=int, default=512, help="bytes per sector (512 eMMC, 4096 UFS)")
    ap.add_argument("new", nargs="*", default=["xnusys=4:linux"], help="NAME=GIB[:linux|apfs], in order after userdata")
    a = ap.parse_args()
    sector, total = a.sector, a.total
    align = ALIGN_BYTES // sector
    wanted = []
    for spec in a.new:
        name, rest = spec.split("=", 1)
        gib, _, kind = rest.partition(":")
        wanted.append((name, int(float(gib) * 2**30) // sector, TYPES[kind or "linux"]))

    pri = bytearray(open(a.primary, "rb").read())
    sec = bytearray(open(a.secondary, "rb").read())
    entlba = parse(pri[sector:2 * sector])["entlba"]
    h, before = table(pri[sector:2 * sector], pri, entlba * sector, sector, "primary")
    esz, nent = h["entsize"], h["nent"]
    assert h["altlba"] == total - 1, f"header says the backup is at {h['altlba']}, the unit ends at {total - 1}"
    sec_base = total - len(sec) // sector                  # the first sector the backup image holds
    sec_entlba = parse(sec[len(sec) - sector:])["entlba"]  # where the backup header says its array is
    off = (sec_entlba - sec_base) * sector
    assert 0 <= off and off + nent * esz <= len(sec) - sector, "the backup array is not inside the backup image"
    sh, sbefore = table(sec[len(sec) - sector:], sec, off, sector, "secondary")
    assert sbefore == before, "the two copies disagree before anything is changed"
    names = [p[2] for p in before]
    for nm, _, _ in wanted:
        assert nm not in names, f"{nm} is in the table already"
    print(f"{len(before)} partitions, sectors of {sector} bytes, last usable sector {h['last']}")

    ents = pri[h["entlba"] * sector: h["entlba"] * sector + nent * esz]
    entries = [bytearray(ents[i * esz:(i + 1) * esz]) for i in range(nent)]
    victim = next(i for i, e in enumerate(entries) if name_of(e) == SHRINK)
    free = [i for i, e in enumerate(entries) if e[0:16] == b"\0" * 16]
    assert len(free) >= len(wanted), "not enough free entries"
    vstart, vend = struct.unpack_from("<QQ", entries[victim], 32)
    show(before, sector, "before:", names[names.index(SHRINK) - 1:names.index(SHRINK) + 2] + [SHRINK])

    # from the end of userdata backwards: the last one asked for lies last, each starts on a 1 MiB boundary
    end, placed = vend, []
    for nm, nsec, typ in reversed(wanted):
        start = (end + 1 - nsec) // align * align
        placed.append((nm, start, end, typ))
        end = start - 1
    placed.reverse()
    assert end > vstart + align, "not enough room in the partition being shrunk"
    for s, t, nm, _ in before:
        for _, ps, pe, _ in placed:
            assert nm == SHRINK or t < ps or s > pe, f"{nm} is in the way ({s}..{t})"

    struct.pack_into("<QQ", entries[victim], 32, vstart, end)
    for (nm, ps, pe, typ), slot in zip(placed, free):
        e = entries[slot]
        e[0:16] = typ.bytes_le
        e[16:32] = uuid.uuid4().bytes_le
        struct.pack_into("<QQQ", e, 32, ps, pe, 0)
        e[56:128] = nm.encode("utf-16-le").ljust(72, b"\0")

    new_ents = b"".join(bytes(x) for x in entries)
    pri[h["entlba"] * sector: h["entlba"] * sector + len(new_ents)] = new_ents
    pri[sector:2 * sector] = build_header(h, new_ents, 1, total - 1, h["entlba"], sector)
    sec[off:off + len(new_ents)] = new_ents
    sec[len(sec) - sector:] = build_header(h, new_ents, total - 1, 1, sec_entlba, sector)

    # read back and checked, both copies, as a firmware would
    h2, after = table(pri[sector:2 * sector], pri, h["entlba"] * sector, sector, "new primary")
    _, safter = table(sec[len(sec) - sector:], sec, off, sector, "new secondary")
    assert after == safter, "the two new copies disagree"
    for (s1, t1, n1, _), (s2, t2, n2, _) in zip(after, after[1:]):
        assert t1 < s2, f"{n1} and {n2} overlap"
    # (only what this made: the phone's own table has recovery at LBA 8, under a "first usable" of 34)
    assert all(h2["first"] <= ps and pe <= h2["last"] for _, ps, pe, _ in placed), "a new partition outside the usable sectors"
    show(after, sector, "after:", names[names.index(SHRINK) - 1:names.index(SHRINK) + 2] + [SHRINK] + [p[0] for p in placed])

    open(a.primary + ".new", "wb").write(bytes(pri))
    open(a.secondary + ".new", "wb").write(bytes(sec))
    print(f"written and checked: {a.primary}.new (sectors 0..{len(pri) // sector - 1}), "
          f"{a.secondary}.new (sectors {total - len(sec) // sector}..{total - 1})")


if __name__ == "__main__":
    main()
