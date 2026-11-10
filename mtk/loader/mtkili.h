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

static void ili_probe(void)
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
		return;
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
		return;
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
}

#endif /* MTKILI_H */
