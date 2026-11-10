#!/usr/bin/env python3
# The ILI7807S's touch firmware (fw-touch/FW_NF_ILI7807S_*.bin, pulled off YOUR phone by pull-firmware.sh) as a C
# header for mtkloader (mtkili.h, ILI_DOWNLOAD=1). The file is a vendor wrapper ("FwUp", up to the first ':') around
# Intel HEX with ILITEK's own records: 0xAF names a block (start, end, number), 0xB0 moves one in the chip's memory.
# Laid out as ILITEK's driver does (read, not copied): blocks AP 1, DATA 2, TUNING 3 are what a normal start loads;
# AP goes to 0, DATA to 0x20610, TUNING right after DATA, unless a 0xB0 record says otherwise. Each block ends in the
# CRC32 (MSB first, polynomial 0x04C11DB7, start 0xFFFFFFFF, no final xor) of the rest, which is checked here.
# The header is the vendor's code: it is never committed (mtk/.gitignore).
# usage: mkilifw.py FIRMWARE OUT.h
import sys

AP, DATA, TUNING = 1, 2, 3
DLM_START = 0x20610
MAX = 160 * 1024


def crc32_msb(data):
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= b << 24
        for _ in range(8):
            crc = ((crc << 1) ^ 0x04C11DB7) if crc & 0x80000000 else crc << 1
            crc &= 0xFFFFFFFF
    return crc


def parse(raw):
    text = raw[raw.index(b":"):].decode("ascii", "replace")
    image = bytearray(b"\xff" * MAX)
    blocks, fixed, ext = {}, {}, 0
    for line in text.splitlines():
        line = line.strip()
        if not line.startswith(":") or len(line) < 11:
            continue
        n, addr, kind = int(line[1:3], 16), int(line[3:7], 16), int(line[7:9], 16)
        data = line[9:9 + 2 * n]
        if kind == 0x04:
            ext = int(data[0:4], 16)
        elif kind == 0x02:
            ext = int(data[0:4], 16) >> 12
        elif kind == 0xAF:
            blocks[int(data[12:14], 16)] = (int(data[0:6], 16), int(data[6:12], 16))
        elif kind == 0xB0:
            fixed[int(data[6:8], 16)] = int(data[0:6], 16)
        elif kind == 0x00:
            at = addr + (ext << 16)
            if at + n > MAX:
                sys.exit("mkilifw: data past %d KB at %#x" % (MAX // 1024, at))
            image[at:at + n] = bytes.fromhex(data)
    return image, blocks, fixed


def main():
    if len(sys.argv) != 3:
        sys.exit("usage: mkilifw.py FIRMWARE OUT.h")
    image, blocks, fixed = parse(open(sys.argv[1], "rb").read())
    for b in (AP, DATA, TUNING):
        if b not in blocks:
            sys.exit("mkilifw: no block %d in %s" % (b, sys.argv[1]))
    mem = {AP: fixed.get(AP, 0), DATA: fixed.get(DATA, DLM_START)}
    mem[TUNING] = fixed.get(TUNING, mem[DATA] + blocks[DATA][1] - blocks[DATA][0] + 1)
    out = ["/* made by mtk/tools/mkilifw.py from %s: the vendor's code, never committed */" % sys.argv[1].split("/")[-1],
           "#ifndef ILIFW_H", "#define ILIFW_H", "#include <stdint.h>",
           "struct ili_block { const char *name; uint32_t mem, len, crc; const uint8_t *data; };"]
    rows = []
    for b, name in ((AP, "AP"), (DATA, "DATA"), (TUNING, "TUNING")):
        start, end = blocks[b]
        body = bytes(image[start:end + 1])
        crc = crc32_msb(body[:-4])
        if crc != int.from_bytes(body[-4:], "big"):
            sys.exit("mkilifw: block %s's CRC %08x is not the one it carries (%s)" % (name, crc, body[-4:].hex()))
        out.append("static const uint8_t ili_%s[%d] = {" % (name.lower(), len(body)))
        for i in range(0, len(body), 24):
            out.append("\t" + ", ".join("0x%02x" % x for x in body[i:i + 24]) + ",")
        out.append("};")
        rows.append('\t{ "%s", 0x%05x, %d, 0x%08x, ili_%s },' % (name, mem[b], len(body), crc, name.lower()))
        print("mkilifw: %-6s hex %06x..%06x -> chip %05x, %6d bytes, CRC %08x (as it carries)" %
              (name, start, end, mem[b], len(body), crc))
    out += ["static const struct ili_block ili_blocks[] = {"] + rows + ["};", "#endif", ""]
    open(sys.argv[2], "w").write("\n".join(out))


if __name__ == "__main__":
    main()
