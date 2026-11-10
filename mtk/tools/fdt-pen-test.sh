#!/bin/bash
# The check for mtkloader's device-tree edit (XNU_PARK, fdt_reserve_pen): its C, taken out of mtk/loader/mtkloader.c,
# run on the host against a device tree as LK passes it (a copy of /sys/firmware/fdt off YOUR phone; not kept in git),
# and the result compared with dtc: the one difference must be the pen's no-map node under /reserved-memory.
# usage: mtk/tools/fdt-pen-test.sh LK.dtb
set -e
HERE="$(cd "$(dirname "$0")/.." && pwd)"
LOADER="$HERE/loader/mtkloader.c"
[ -f "$LOADER" ] || LOADER="$HERE/mtkloader/mtkloader.c"        # the device repository's layout
[ -f "${1:-}" ] || { echo "usage: $0 LK.dtb" >&2; exit 1; }
command -v dtc >/dev/null || { echo "fdt-pen-test: needs dtc" >&2; exit 1; }
T=$(mktemp -d)
trap 'rm -rf "$T"' EXIT
sed -n "/^\/\* ---- LK.s device tree: one node added/,/^\/\* the pen copied to PARK_PA/p" "$LOADER" | sed '$d' |
	sed 's/(uint32_t)PARK_PA)/(uint32_t)PARK_PA_REAL)/; s/out_hex(PARK_PA);/out_hex(PARK_PA_REAL);/' > "$T/fdt_part.c"
cat > "$T/harness.c" <<'EOF'
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
static int memcmp_n(const volatile void *a, const void *b, size_t n)
{
	const volatile uint8_t *x = a;
	const uint8_t *y = b;
	for (size_t i = 0; i < n; i++)
		if (x[i] != y[i])
			return 1;
	return 0;
}
static void out(const char *s) { fputs(s, stdout); }
static void out_hex(uint64_t v) { printf("%#llx", (unsigned long long)v); }
static void out_dec(uint64_t v) { printf("%llu", (unsigned long long)v); }
#define PARK_PA_REAL 0x4cdff000ULL
#define PARK_NODE "hakios-pen@4cdff000"
#define PARK_NODE_BYTES 68
static uint64_t PARK_PA;        /* the room check against the end of the host's buffer instead */
#include "fdt_part.c"
int main(int argc, char **argv)
{
	FILE *f = fopen(argv[1], "rb");
	long n;
	uint8_t *buf;

	if (argc != 3 || f == NULL)
		return 2;
	fseek(f, 0, SEEK_END);
	n = ftell(f);
	rewind(f);
	buf = calloc(1, n + 4096);
	if (fread(buf, 1, n, f) != (size_t)n)
		return 2;
	fclose(f);
	PARK_PA = (uint64_t)(uintptr_t)buf + n + 4096;
	if (!fdt_pen_fits((uint64_t)(uintptr_t)buf))
		return 1;
	fdt_reserve_pen((uint64_t)(uintptr_t)buf);
	f = fopen(argv[2], "wb");
	fwrite(buf, 1, fdt_get((uint64_t)(uintptr_t)buf + 4), f);
	fclose(f);
	return 0;
}
EOF
cc -Wall -Wextra -o "$T/harness" "$T/harness.c"
"$T/harness" "$1" "$T/out.dtb"
dtc -q -I dtb -O dts "$1" > "$T/before.dts"
dtc -q -I dtb -O dts "$T/out.dtb" > "$T/after.dts"
diff "$T/before.dts" "$T/after.dts" > "$T/diff" || true
cat > "$T/want" <<'EOF'
>
> 		hakios-pen@4cdff000 {
> 			reg = <0x00 0x4cdff000 0x00 0x1000>;
> 			no-map;
> 		};
EOF
if [ "$(grep '^>' "$T/diff" | sed 's/[[:space:]]*$//')" = "$(sed 's/[[:space:]]*$//' "$T/want")" ] && ! grep -q '^<' "$T/diff"; then
	echo "fdt-pen-test: ok, the one change is the pen's node under /reserved-memory"
else
	echo "fdt-pen-test: FAILED:" >&2
	cat "$T/diff" >&2
	exit 1
fi
