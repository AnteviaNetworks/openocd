// SPDX-License-Identifier: GPL-2.0-or-later
/***************************************************************************
 *   Based on u-boot file drivers/spi/fsl_qspi.c                           *
 *     Distrubuted under license:                                          *
 *	                                                                   *
 *     * Copyright 2013-2015 Freescale Semiconductor, Inc.         *       *
 *     *                                                           *       *
 *     * Freescale Quad Serial Peripheral Interface (QSPI) driver  *       *
 *     *                                                           *       *
 *     * SPDX-License-Identifier:     GPL-2.0+                     *       *
 *	                                                                   *
 *     OpenOCD fslqspi uses LE register values (same as u-boot)            *
 *     The Layerscape QuadSPI register set is Big Endian)                  *
 *   Copyright (C) 2023 by Steve Williams                                  *
 *   steve.williams@telxio.com                                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "fslqspi.h"
#include <helper/time_support.h>

#define RX_BUFFER_SIZE          0x80
#define TX_BUFFER_SIZE          0x40

#define FLASH_STATUS_WEL        0x02
#define FLASH_STATUS_QE         BIT(1)


/* SEQID */
#define SEQID_WREN              1
#define SEQID_FAST_READ         2
#define SEQID_RDSR              3
#define SEQID_SE                4
#define SEQID_CHIP_ERASE        5
#define SEQID_PP                6
#define SEQID_RDID              7
#define SEQID_BE_4K             8
#ifdef CONFIG_SPI_FLASH_BAR
#define SEQID_BRRD              9
#define SEQID_BRWR              10
#define SEQID_RDEAR             11
#define SEQID_WREAR             12
#endif
#define SEQID_WRAR              13
#define SEQID_RDAR              14
//#define SEQID_RDSR2             15
//#define SEQID_WRSR              16

/* QSPI CMD */
#define QSPI_CMD_PP             0x02    /* Page program (up to 256 bytes) */
//#define QSPI_CMD_WRSR           0x05    /* Write status register */
#define QSPI_CMD_RDSR           0x05    /* Read status register */
//#define QSPI_CMD_RDSR2          0x35    /* Read status register 2 */
#define QSPI_CMD_WREN           0x06    /* Write enable */
#define QSPI_CMD_FAST_READ      0x0b    /* Read data bytes (high frequency) */
#define QSPI_CMD_BE_4K          0x20    /* 4K erase */
#define QSPI_CMD_CHIP_ERASE     0xc7    /* Erase whole flash chip */
#define QSPI_CMD_SE             0xd8    /* Sector erase (usually 64KiB) */
#define QSPI_CMD_RDID           0x9f    /* Read JEDEC ID */

/* Used for Micron, winbond and Macronix flashes */
#define QSPI_CMD_WREAR          0xc5    /* EAR register write */
#define QSPI_CMD_RDEAR          0xc8    /* EAR reigster read */

/* Used for Spansion flashes only. */
#define QSPI_CMD_BRRD           0x16    /* Bank register read */
#define QSPI_CMD_BRWR           0x17    /* Bank register write */

/* Used for Spansion S25FS-S family flash only. */
#define QSPI_CMD_RDAR           0x65    /* Read any device register */
#define QSPI_CMD_WRAR           0x71    /* Write any device register */

/* 4-byte address QSPI CMD - used on Spansion and some Macronix flashes */
#define QSPI_CMD_FAST_READ_4B   0x0c    /* Read data bytes (high frequency) */
#define QSPI_CMD_PP_4B          0x12    /* Page program (up to 256 bytes) */
#define QSPI_CMD_SE_4B          0xdc    /* Sector erase (usually 64KiB) */

#define REG(p) (target_addr_t)((uintptr_t)(p))

#define RERR_READ32(t, r, pv) \
	do { \
		int rv; \
		if((rv = qspi_read32(t, REG(r), pv)) != ERROR_OK) \
			return rv; \
	} while(0)

#define RERR_WRITE32(t, r, v) \
	do { \
		int rv; \
		if((rv = qspi_write32((t), REG(r), (v))) != ERROR_OK) \
			return rv; \
	} while(0)

#define RERR_WAIT32(tgt, r, m, s, to) \
	do { \
		int rv; \
		if((rv = wait_for_bit((tgt), (r), (m), (s), (to))) != ERROR_OK) \
			return rv; \
	} while(0)

static inline uint16_t swab16(uint16_t val)
{
	return ___swab16(val);
}

static inline uint32_t swab32(uint32_t val)
{
	return ___swab32(val);
}

static inline uint64_t swab64(uint64_t val)
{
	return ___swab64(val);
}

static uint32_t qspi_read32(struct target* target, target_addr_t reg, uint32_t* val)
{
	uint32_t value, retval;
	retval = target_read_u32(target, reg, &value);
	if(retval != ERROR_OK)
		value = 0;
	if(target->endianness == TARGET_BIG_ENDIAN)
		value = swab32(value);
	*val = value;
	return retval;
}

static uint32_t qspi_write32(struct target* target, target_addr_t reg, uint32_t val)
{
	uint32_t retval;
	if(target->endianness == TARGET_BIG_ENDIAN)
		val = swab32(val);
	retval = target_write_u32(target, reg, val);
	return retval;
}

static uint32_t qspi_endian_xchg(uint32_t data)
{
	return swab32(data);
}

static int wait_for_bit(struct target* target, uint32_t* reg, const uint32_t mask,
	    const bool set, const unsigned int timeout_ms)
{
	uint32_t val;
	unsigned long start = timeval_ms();

	while (1) {
		RERR_READ32(target, reg, &val);

		if (!set)
			val = ~val;

		if ((val & mask) == mask)
			return 0;

		if (timeval_ms() - start > timeout_ms)
			break;

		alive_sleep(1);
	}
	return -ETIMEDOUT;
}

static int qspi_set_lut(struct fslqspi_flash_bank *priv)
{
	struct fslqspi_regs *regs = priv->regs;
	uint32_t lut_base;

	/* Unlock the LUT */
	RERR_WRITE32(priv->target, &regs->lutkey, LUT_KEY_VALUE);
	RERR_WRITE32(priv->target, &regs->lckcr, QSPI_LCKCR_UNLOCK);

	/* Write Enable */
	lut_base = SEQID_WREN * 4;
	RERR_WRITE32(priv->target, &regs->lut[lut_base], OPRND0(QSPI_CMD_WREN) |
		PAD0(LUT_PAD1) | INSTR0(LUT_CMD));
	RERR_WRITE32(priv->target, &regs->lut[lut_base + 1], 0);
	RERR_WRITE32(priv->target, &regs->lut[lut_base + 2], 0);
	RERR_WRITE32(priv->target, &regs->lut[lut_base + 3], 0);

	/* Fast Read */
	lut_base = SEQID_FAST_READ * 4;
	if (priv->amba_size  <= SZ_16M)
		RERR_WRITE32(priv->target, &regs->lut[lut_base],
			     OPRND0(QSPI_CMD_FAST_READ) | PAD0(LUT_PAD1) |
			     INSTR0(LUT_CMD) | OPRND1(ADDR24BIT) |
			     PAD1(LUT_PAD1) | INSTR1(LUT_ADDR));
	else
		RERR_WRITE32(priv->target, &regs->lut[lut_base],
			     OPRND0(QSPI_CMD_FAST_READ_4B) |
			     PAD0(LUT_PAD1) | INSTR0(LUT_CMD) |
			     OPRND1(ADDR32BIT) | PAD1(LUT_PAD1) |
			     INSTR1(LUT_ADDR));
	RERR_WRITE32(priv->target, &regs->lut[lut_base + 1],
		     OPRND0(8) | PAD0(LUT_PAD1) | INSTR0(LUT_DUMMY) |
		     OPRND1(RX_BUFFER_SIZE) | PAD1(LUT_PAD1) |
		     INSTR1(LUT_READ));
	RERR_WRITE32(priv->target, &regs->lut[lut_base + 2], 0);
	RERR_WRITE32(priv->target, &regs->lut[lut_base + 3], 0);

	/* Write Status */
	//lut_base = SEQID_WRSR * 4;
	//RERR_WRITE32(priv->target, &regs->lut[lut_base], OPRND0(QSPI_CMD_WRSR) |
//		PAD0(LUT_PAD1) | INSTR0(LUT_CMD) | OPRND1(2) |
//		PAD1(LUT_PAD1) | INSTR1(LUT_READ));
	//RERR_WRITE32(priv->target, &regs->lut[lut_base + 1], 0);
	//RERR_WRITE32(priv->target, &regs->lut[lut_base + 2], 0);
	//RERR_WRITE32(priv->target, &regs->lut[lut_base + 3], 0);

	/* Read Status */
	lut_base = SEQID_RDSR * 4;
	RERR_WRITE32(priv->target, &regs->lut[lut_base], OPRND0(QSPI_CMD_RDSR) |
		PAD0(LUT_PAD1) | INSTR0(LUT_CMD) | OPRND1(1) |
		PAD1(LUT_PAD1) | INSTR1(LUT_READ));
	RERR_WRITE32(priv->target, &regs->lut[lut_base + 1], 0);
	RERR_WRITE32(priv->target, &regs->lut[lut_base + 2], 0);
	RERR_WRITE32(priv->target, &regs->lut[lut_base + 3], 0);

	/* Read Status 2 */
	//lut_base = SEQID_RDSR2 * 4;
	//RERR_WRITE32(priv->target, &regs->lut[lut_base], OPRND0(QSPI_CMD_RDSR2) |
//		PAD0(LUT_PAD1) | INSTR0(LUT_CMD) | OPRND1(1) |
//		PAD1(LUT_PAD1) | INSTR1(LUT_READ));
	//RERR_WRITE32(priv->target, &regs->lut[lut_base + 1], 0);
	//RERR_WRITE32(priv->target, &regs->lut[lut_base + 2], 0);
	//RERR_WRITE32(priv->target, &regs->lut[lut_base + 3], 0);

	/* Erase a sector */
	lut_base = SEQID_SE * 4;
	if (priv->amba_size  <= SZ_16M)
		RERR_WRITE32(priv->target, &regs->lut[lut_base],
			     OPRND0(QSPI_CMD_SE) | PAD0(LUT_PAD1) |
			     INSTR0(LUT_CMD) | OPRND1(ADDR24BIT) |
			     PAD1(LUT_PAD1) | INSTR1(LUT_ADDR));
	else
		RERR_WRITE32(priv->target, &regs->lut[lut_base],
			     OPRND0(QSPI_CMD_SE_4B) | PAD0(LUT_PAD1) |
			     INSTR0(LUT_CMD) | OPRND1(ADDR32BIT) |
			     PAD1(LUT_PAD1) | INSTR1(LUT_ADDR));
	RERR_WRITE32(priv->target, &regs->lut[lut_base + 1], 0);
	RERR_WRITE32(priv->target, &regs->lut[lut_base + 2], 0);
	RERR_WRITE32(priv->target, &regs->lut[lut_base + 3], 0);

	/* Erase the whole chip */
	lut_base = SEQID_CHIP_ERASE * 4;
	RERR_WRITE32(priv->target, &regs->lut[lut_base],
		     OPRND0(QSPI_CMD_CHIP_ERASE) |
		     PAD0(LUT_PAD1) | INSTR0(LUT_CMD));
	RERR_WRITE32(priv->target, &regs->lut[lut_base + 1], 0);
	RERR_WRITE32(priv->target, &regs->lut[lut_base + 2], 0);
	RERR_WRITE32(priv->target, &regs->lut[lut_base + 3], 0);

	/* Page Program */
	lut_base = SEQID_PP * 4;
	if (priv->amba_size  <= SZ_16M)
		RERR_WRITE32(priv->target, &regs->lut[lut_base],
			     OPRND0(QSPI_CMD_PP) | PAD0(LUT_PAD1) |
			     INSTR0(LUT_CMD) | OPRND1(ADDR24BIT) |
			     PAD1(LUT_PAD1) | INSTR1(LUT_ADDR));
	else
		RERR_WRITE32(priv->target, &regs->lut[lut_base],
			     OPRND0(QSPI_CMD_PP_4B) | PAD0(LUT_PAD1) |
			     INSTR0(LUT_CMD) | OPRND1(ADDR32BIT) |
			     PAD1(LUT_PAD1) | INSTR1(LUT_ADDR));
	RERR_WRITE32(priv->target, &regs->lut[lut_base + 1],
		     OPRND0(TX_BUFFER_SIZE) |
		     PAD0(LUT_PAD1) | INSTR0(LUT_WRITE));
	RERR_WRITE32(priv->target, &regs->lut[lut_base + 2], 0);
	RERR_WRITE32(priv->target, &regs->lut[lut_base + 3], 0);

	/* READ ID */
	lut_base = SEQID_RDID * 4;
	RERR_WRITE32(priv->target, &regs->lut[lut_base], OPRND0(QSPI_CMD_RDID) |
		PAD0(LUT_PAD1) | INSTR0(LUT_CMD) | OPRND1(8) |
		PAD1(LUT_PAD1) | INSTR1(LUT_READ));
	RERR_WRITE32(priv->target, &regs->lut[lut_base + 1], 0);
	RERR_WRITE32(priv->target, &regs->lut[lut_base + 2], 0);
	RERR_WRITE32(priv->target, &regs->lut[lut_base + 3], 0);

	/* SUB SECTOR 4K ERASE */
	lut_base = SEQID_BE_4K * 4;
	RERR_WRITE32(priv->target, &regs->lut[lut_base], OPRND0(QSPI_CMD_BE_4K) |
		     PAD0(LUT_PAD1) | INSTR0(LUT_CMD) | OPRND1(ADDR24BIT) |
		     PAD1(LUT_PAD1) | INSTR1(LUT_ADDR));

	/*
	 * Read any device register.
	 * Used for Spansion S25FS-S family flash only.
	 */
	lut_base = SEQID_RDAR * 4;
	RERR_WRITE32(priv->target, &regs->lut[lut_base],
		     OPRND0(QSPI_CMD_RDAR) | PAD0(LUT_PAD1) |
		     INSTR0(LUT_CMD) | OPRND1(ADDR24BIT) |
		     PAD1(LUT_PAD1) | INSTR1(LUT_ADDR));
	RERR_WRITE32(priv->target, &regs->lut[lut_base + 1],
		     OPRND0(8) | PAD0(LUT_PAD1) | INSTR0(LUT_DUMMY) |
		     OPRND1(1) | PAD1(LUT_PAD1) |
		     INSTR1(LUT_READ));

	/*
	 * Write any device register.
	 * Used for Spansion S25FS-S family flash only.
	 */
	lut_base = SEQID_WRAR * 4;
	RERR_WRITE32(priv->target, &regs->lut[lut_base],
		     OPRND0(QSPI_CMD_WRAR) | PAD0(LUT_PAD1) |
		     INSTR0(LUT_CMD) | OPRND1(ADDR24BIT) |
		     PAD1(LUT_PAD1) | INSTR1(LUT_ADDR));
	RERR_WRITE32(priv->target, &regs->lut[lut_base + 1],
		     OPRND0(1) | PAD0(LUT_PAD1) | INSTR0(LUT_WRITE));

	/* Lock the LUT */
	RERR_WRITE32(priv->target, &regs->lutkey, LUT_KEY_VALUE);
	RERR_WRITE32(priv->target, &regs->lckcr, QSPI_LCKCR_LOCK);

	return ERROR_OK;
}

/*
static int fslqspi_set_qe_bit_winbond_spansion(struct fslqspi_flash_bank *priv) {
}

static int fslqspi_quad_enable(struct fslqspi_flash_bank *priv)
{
	int retval;
	switch(JEDEC_MFR(priv->info)) {
		case MFR_SPANSION:
		case MFR_WINBOND:
			retval = fslqspi_set_qe_bit_winbond_spansion(priv);
			break;
		default:
			LOG_ERROR("Need function to set QE bit for flash manufacturer 0x%02x",
				JEDEC_MFR(priv->info));
		retval = ERROR_FAIL;
	}
	return retval;
}
*/

static int qspi_module_disable(struct fslqspi_flash_bank *priv, uint8_t disable)
{
        uint32_t mcr_val;

        RERR_READ32(priv->target, &priv->regs->mcr, &mcr_val);
        if (disable)
                mcr_val |= QSPI_MCR_MDIS_MASK;
        else
                mcr_val &= ~QSPI_MCR_MDIS_MASK;
        RERR_WRITE32(priv->target, &priv->regs->mcr, mcr_val);
	return ERROR_OK;
}

static int qspi_check_protect(struct fslqspi_flash_bank *priv, uint8_t *enabled)
{
	struct fslqspi_regs *regs = priv->regs;
        uint32_t mcr_reg, rbsr_reg, rbdr_reg;

        RERR_READ32(priv->target, &regs->mcr, &mcr_reg);
        RERR_WRITE32(priv->target, &regs->mcr,
                     QSPI_MCR_CLR_RXF_MASK | QSPI_MCR_CLR_TXF_MASK |
                     QSPI_MCR_RESERVED_MASK | QSPI_MCR_END_CFD_LE);
        RERR_WRITE32(priv->target, &regs->rbct, QSPI_RBCT_RXBRD_USEIPS);

	RERR_WRITE32(priv->target, &regs->ipcr,
		     (SEQID_RDSR << QSPI_IPCR_SEQID_SHIFT) | 1);
	RERR_WAIT32(priv->target, &priv->regs->sr, QSPI_SR_BUSY_MASK, false, 100);

	RERR_READ32(priv->target, &regs->rbsr, &rbsr_reg);
	if (rbsr_reg & QSPI_RBSR_RDBFL_MASK) {
		RERR_READ32(priv->target, &regs->rbdr[0], &rbdr_reg);
		*enabled = (rbdr_reg & FLASH_STATUS_WEL) != FLASH_STATUS_WEL ? 1 : 0;
	}
	RERR_WRITE32(priv->target, &regs->mcr, mcr_reg);
	return ERROR_OK;
}
/*
static int qspi_op_rd_qeen(struct fslqspi_flash_bank *priv, uint8_t* sr2)
{
	struct fslqspi_regs *regs = priv->regs;
	uint32_t mcr_reg, rbsr_reg, rbdr_reg;
	uint8_t cmd = QSPI_CMD_RDSR2;

        ret = read_sr(flash, &qeb_status);

#define CMD_READ_STATUS                 0x05

	static int read_sr(struct spi_flash *flash, u8 *rs)
	{
		int ret;
		u8 cmd;

		cmd = CMD_READ_STATUS;
		ret = spi_flash_read_common(flash, &cmd, 1, rs, 1);
		if (ret < 0) {
			debug("SF: fail to read status register\n");
			return ret;
		}

		return 0;
	}

	int spi_flash_read_common(struct spi_flash *flash, const u8 *cmd,
			size_t cmd_len, void *data, size_t data_len)
	{
		struct spi_slave *spi = flash->spi;
		int ret;

		ret = spi_claim_bus(spi);
		if (ret) {
			debug("SF: unable to claim SPI bus\n");
			return ret;
		}

		ret = spi_flash_cmd_read(spi, cmd, cmd_len, data, data_len);
		if (ret < 0) {
			debug("SF: read cmd failed\n");
			return ret;
		}

		spi_release_bus(spi);

		return ret;
	}

	int spi_flash_cmd_read(struct spi_slave *spi, const u8 *cmd,
			size_t cmd_len, void *data, size_t data_len)
	{
		return spi_flash_read_write(spi, cmd, cmd_len, NULL, data, data_len);
	}


	static int spi_flash_read_write(struct spi_slave *spi,
					const u8 *cmd, size_t cmd_len,
					const u8 *data_out, u8 *data_in,
					size_t data_len)
	{
		unsigned long flags = SPI_XFER_BEGIN;
		int ret;

		if (data_len == 0)
			flags |= SPI_XFER_END;

		ret = spi_xfer(spi, cmd_len * 8, cmd, NULL, flags);
		if (ret) {
			debug("SF: Failed to send command (%zu bytes): %d\n",
			      cmd_len, ret);
		} else if (data_len != 0) {
			ret = spi_xfer(spi, data_len * 8, data_out, data_in,
						SPI_XFER_END);
			if (ret)
				debug("SF: Failed to transfer %zu bytes of data: %d\n",
				      data_len, ret);
		}

		return ret;
	}




        RERR_READ32(priv->target, &regs->mcr, &mcr_reg);
        RERR_WRITE32(priv->target, &regs->mcr,
                     QSPI_MCR_CLR_RXF_MASK | QSPI_MCR_CLR_TXF_MASK |
                     QSPI_MCR_RESERVED_MASK | QSPI_MCR_END_CFD_LE);
        RERR_WRITE32(priv->target, &regs->rbct, QSPI_RBCT_RXBRD_USEIPS);
	RERR_WRITE32(priv->target, &regs->ipcr,
		     (SEQID_RDSR2 << QSPI_IPCR_SEQID_SHIFT) | 1);
	RERR_WAIT32(priv->target, &priv->regs->sr, QSPI_SR_BUSY_MASK, false, 100);

	RERR_READ32(priv->target, &regs->rbsr, &rbsr_reg);
	if (rbsr_reg & QSPI_RBSR_RDBFL_MASK) {
		RERR_READ32(priv->target, &regs->rbdr[0], &rbdr_reg);
		//rbdr_reg = qspi_endian_xchg(rbdr_reg);
	}
	if ((rbdr_reg & FLASH_STATUS_QE) != FLASH_STATUS_QE)
	{
	}

        while ((rbdr_reg & FLASH_STATUS_QE) != FLASH_STATUS_QE) {
		uint32_t tmp;
                RERR_WRITE32(priv->target, &regs->ipcr,
                             (SEQID_RDSR2 << QSPI_IPCR_SEQID_SHIFT) | FLASH_STATUS_QE);
		RERR_WAIT32(priv->target, &priv->regs->sr, QSPI_SR_BUSY_MASK, false, 100);

		RERR_READ32(priv->target, &regs->mcr, &tmp);
                RERR_WRITE32(priv->target, &regs->mcr, tmp | QSPI_MCR_CLR_RXF_MASK);
        }
	RERR_WRITE32(priv->target, &regs->mcr, mcr_reg);
	return ERROR_OK;
}

static int qspi_op_wr_qeen(struct fslqspi_flash_bank *priv, uint8_t sr2)
{
	struct fslqspi_regs *regs = priv->regs;
	uint32_t mcr_reg, rbsr_reg, rbdr_reg;
	uint8_t cmd = QSPI_CMD_WRSR;
	uint8_t data[2] = {0, sr | FLASH_STATUS_QE};

        RERR_READ32(priv->target, &regs->mcr, &mcr_reg);
        RERR_WRITE32(priv->target, &regs->mcr,
                     QSPI_MCR_CLR_RXF_MASK | QSPI_MCR_CLR_TXF_MASK |
                     QSPI_MCR_RESERVED_MASK | QSPI_MCR_END_CFD_LE);
        RERR_WRITE32(priv->target, &regs->rbct, QSPI_RBCT_RXBRD_USEIPS);
	RERR_WRITE32(priv->target, &regs->ipcr,
		     (SEQID_RDSR2 << QSPI_IPCR_SEQID_SHIFT) | 1);
	RERR_WAIT32(priv->target, &priv->regs->sr, QSPI_SR_BUSY_MASK, false, 100);

	RERR_READ32(priv->target, &regs->rbsr, &rbsr_reg);
	if (rbsr_reg & QSPI_RBSR_RDBFL_MASK) {
		RERR_READ32(priv->target, &regs->rbdr[0], &rbdr_reg);
		//rbdr_reg = qspi_endian_xchg(rbdr_reg);
	}
	if ((rbdr_reg & FLASH_STATUS_QE) != FLASH_STATUS_QE)
	{
	}

        while ((rbdr_reg & FLASH_STATUS_QE) != FLASH_STATUS_QE) {
		uint32_t tmp;
                RERR_WRITE32(priv->target, &regs->ipcr,
                             (SEQID_RDSR2 << QSPI_IPCR_SEQID_SHIFT) | FLASH_STATUS_QE);
		RERR_WAIT32(priv->target, &priv->regs->sr, QSPI_SR_BUSY_MASK, false, 100);

		RERR_READ32(priv->target, &regs->mcr, &tmp);
                RERR_WRITE32(priv->target, &regs->mcr, tmp | QSPI_MCR_CLR_RXF_MASK);
        }
	RERR_WRITE32(priv->target, &regs->mcr, mcr_reg);
	return ERROR_OK;
}
*/
static int qspi_op_wren(struct fslqspi_flash_bank *priv, uint8_t enable)
{
	struct fslqspi_regs *regs = priv->regs;
	uint32_t mcr_reg, rbsr_reg, rbdr_reg;

        RERR_READ32(priv->target, &regs->mcr, &mcr_reg);
        RERR_WRITE32(priv->target, &regs->mcr,
                     QSPI_MCR_CLR_RXF_MASK | QSPI_MCR_CLR_TXF_MASK |
                     QSPI_MCR_RESERVED_MASK | QSPI_MCR_END_CFD_LE);
        RERR_WRITE32(priv->target, &regs->rbct, QSPI_RBCT_RXBRD_USEIPS);
        rbdr_reg = 0;
        while ((rbdr_reg & FLASH_STATUS_WEL) != FLASH_STATUS_WEL) {
		uint32_t tmp;
                RERR_WRITE32(priv->target, &regs->ipcr,
                             (SEQID_WREN << QSPI_IPCR_SEQID_SHIFT) | 0);
		RERR_WAIT32(priv->target, &priv->regs->sr, QSPI_SR_BUSY_MASK, false, 100);

                RERR_WRITE32(priv->target, &regs->ipcr,
                             (SEQID_RDSR << QSPI_IPCR_SEQID_SHIFT) | 1);
		RERR_WAIT32(priv->target, &priv->regs->sr, QSPI_SR_BUSY_MASK, false, 100);

                RERR_READ32(priv->target, &regs->rbsr, &rbsr_reg);
                if (rbsr_reg & QSPI_RBSR_RDBFL_MASK) {
                        RERR_READ32(priv->target, &regs->rbdr[0], &rbdr_reg);
                        //rbdr_reg = qspi_endian_xchg(rbdr_reg);
                }
		RERR_READ32(priv->target, &regs->mcr, &tmp);
                RERR_WRITE32(priv->target, &regs->mcr, tmp | QSPI_MCR_CLR_RXF_MASK);
        }
	RERR_WRITE32(priv->target, &regs->mcr, mcr_reg);
	return ERROR_OK;
}

static int qspi_op_rdid(struct fslqspi_flash_bank *priv, uint32_t *rxbuf, uint32_t len)
{
	struct fslqspi_regs *regs = priv->regs;
	uint32_t mcr_reg, rbsr_reg, data, size;
	int i;

	RERR_READ32(priv->target, &regs->mcr, &mcr_reg);
	RERR_WRITE32(priv->target, &regs->mcr,
		     QSPI_MCR_CLR_RXF_MASK | QSPI_MCR_CLR_TXF_MASK |
		     QSPI_MCR_RESERVED_MASK | QSPI_MCR_END_CFD_LE);
	RERR_WRITE32(priv->target, &regs->rbct, QSPI_RBCT_RXBRD_USEIPS);

	RERR_WRITE32(priv->target, &regs->sfar, priv->cur_amba_base);

	RERR_WRITE32(priv->target, &regs->ipcr,
		     (SEQID_RDID << QSPI_IPCR_SEQID_SHIFT) | 0);
        RERR_WAIT32(priv->target, &priv->regs->sr, QSPI_SR_BUSY_MASK, false, 100);

	i = 0;
	while ((RX_BUFFER_SIZE >= len) && (len > 0)) {
		RERR_READ32(priv->target, &regs->rbsr, &rbsr_reg);
		if (rbsr_reg & QSPI_RBSR_RDBFL_MASK) {
			RERR_READ32(priv->target, &regs->rbdr[i], &data);
			//data = qspi_endian_xchg(data);
			size = (len < 4) ? len : 4;
			memcpy(rxbuf, &data, size);
			len -= size;
			rxbuf++;
			i++;
		}
	}

	RERR_WRITE32(priv->target, &regs->mcr, mcr_reg);
	return ERROR_OK;
}

static int qspi_op_read(struct fslqspi_flash_bank *priv, uint32_t *rxbuf, uint32_t len)
{
	struct fslqspi_regs *regs = priv->regs;
	uint32_t mcr_reg, data;
	int i, size;
	uint32_t to_or_from;
	uint32_t seqid;

	//if (priv->cur_seqid == QSPI_CMD_RDAR)
	//	seqid = SEQID_RDAR;
	//else
		seqid = SEQID_FAST_READ;

	RERR_READ32(priv->target, &regs->mcr, &mcr_reg);
	RERR_WRITE32(priv->target, &regs->mcr,
		     QSPI_MCR_CLR_RXF_MASK | QSPI_MCR_CLR_TXF_MASK |
		     QSPI_MCR_RESERVED_MASK | QSPI_MCR_END_CFD_LE);
	RERR_WRITE32(priv->target, &regs->rbct, QSPI_RBCT_RXBRD_USEIPS);

	to_or_from = priv->sf_addr + priv->cur_amba_base;

	while (len > 0) {
		uint32_t tmp;
		to_or_from = qspi_endian_xchg(to_or_from);
		to_or_from = qspi_endian_xchg(to_or_from);
		LOG_DEBUG("loading read address SFAR(%p): 0x%08x", &regs->sfar, to_or_from);
		RERR_WRITE32(priv->target, &regs->sfar, to_or_from);

		size = (len > RX_BUFFER_SIZE) ?
			RX_BUFFER_SIZE : len;
		tmp = (seqid << QSPI_IPCR_SEQID_SHIFT) | size;
		LOG_DEBUG("sending read command IPCR(%p): 0x%08x", &regs->ipcr, tmp);
		RERR_WRITE32(priv->target, &regs->ipcr,
			     (seqid << QSPI_IPCR_SEQID_SHIFT) |
			     size);
		RERR_WAIT32(priv->target, &regs->sr, QSPI_SR_BUSY_MASK, false, 100);

		to_or_from += size;
		len -= size;

		i = 0;
		while ((RX_BUFFER_SIZE >= size) && (size > 0)) {
			LOG_DEBUG("reading data word %d", i);
			RERR_READ32(priv->target, &regs->rbdr[i], &data);
			//data = qspi_endian_xchg(data);
			if (size < 4)
				memcpy(rxbuf, &data, size);
			else
				memcpy(rxbuf, &data, 4);
			rxbuf++;
			size -= 4;
			i++;
		}
		RERR_READ32(priv->target, &regs->mcr, &tmp);
                RERR_WRITE32(priv->target, &regs->mcr, tmp | QSPI_MCR_CLR_RXF_MASK);
	}

	RERR_WRITE32(priv->target, &regs->mcr, mcr_reg);
	return ERROR_OK;
}

int fslqspi_erase(struct flash_bank *bank, unsigned int first,
                unsigned int last)
{
	return ERROR_OK;
}

int fslqspi_protect(struct flash_bank *bank, int set, unsigned int first,
                unsigned int last)
{
        struct fslqspi_flash_bank *priv = bank->driver_priv;

	LOG_DEBUG("protect %s blocks first %u to last %u!", set ? "on" : "off", first, last);
        if (bank->target->state != TARGET_HALTED) {
                LOG_ERROR("Target not halted");
                return ERROR_TARGET_NOT_HALTED;
        }

        if (!priv->probed)
                return ERROR_FLASH_BANK_NOT_PROBED;

	if(set)
		LOG_WARNING("protect: fslqspi primary command set unsupported");

	return qspi_op_wren(priv, !set);
}

static int fslqspi_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	return ERROR_OK;
}

static int fslqspi_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
        struct fslqspi_flash_bank *priv = bank->driver_priv;

        LOG_DEBUG("reading buffer of %i byte at 0x%8.8x",
                (int)count, (unsigned)offset);

        if (bank->target->state != TARGET_HALTED) {
                LOG_ERROR("Target not halted");
                return ERROR_TARGET_NOT_HALTED;
        }

        if (offset + count > bank->size)
                return ERROR_FLASH_DST_OUT_OF_BANK;

        if (!priv->probed)
                return ERROR_FLASH_BANK_NOT_PROBED;

	priv->sf_addr = offset;
	return qspi_op_read(priv, (uint32_t*)buffer, count);
}

int fslqspi_probe(struct flash_bank *bank)
{
        struct fslqspi_flash_bank *priv = bank->driver_priv;
	struct fslqspi_regs *regs;
	int retval;
	uint32_t mcr_val, smpr_val;
	uint32_t amba_size_per_chip;
	uint32_t offset = 0;

        if (bank->target->state != TARGET_HALTED) {
                LOG_ERROR("Target not halted");
                return ERROR_TARGET_NOT_HALTED;
        }

        priv->probed = false;
        priv->num_erase_regions = 0;

	if(bank->sectors)
		free(bank->sectors);
        bank->sectors = NULL;
	bank->num_sectors = 0;

        if(priv->erase_region_info)
		free(priv->erase_region_info);
        priv->erase_region_info = NULL;

        LOG_DEBUG("probing FSL QSPI at flask bank %d address 0x%016lx",
		bank->bank_number, bank->base);
        priv->target = bank->target;
        regs = priv->regs = (struct fslqspi_regs *)(uintptr_t)priv->reg_base;

        /* make sure controller is not busy anywhere */
        RERR_WAIT32(priv->target, &regs->sr,
			QSPI_SR_BUSY_MASK |
			QSPI_SR_AHB_ACC_MASK |
			QSPI_SR_IP_ACC_MASK,
			false, 100);

        RERR_READ32(priv->target, &regs->mcr, &mcr_val);

        RERR_WRITE32(priv->target, &regs->mcr,
                     QSPI_MCR_RESERVED_MASK | QSPI_MCR_MDIS_MASK |
                     (mcr_val & QSPI_MCR_END_CFD_MASK));

        RERR_READ32(priv->target, &regs->smpr, &smpr_val);
        smpr_val &= (QSPI_SMPR_FSDLY_MASK | QSPI_SMPR_DDRSMP_MASK |
                      QSPI_SMPR_FSPHS_MASK | QSPI_SMPR_HSENA_MASK);
        RERR_WRITE32(priv->target, &regs->smpr, smpr_val);

	/*
	 * divide the flash memory block between the
	 * available chip selects, only 1,2 and 4 devices
	 * are supported.
	 */
        amba_size_per_chip = priv->amba_size >>
                             (priv->num_cs >> 1);
        for (int i = 1 ; i < priv->num_cs ; i++)
                priv->amba_base[i] =
                        amba_size_per_chip + priv->amba_base[i - 1];

        RERR_WRITE32(priv->target, &regs->sfa1ad,
                     priv->amba_base[0] | amba_size_per_chip);
        switch (priv->num_cs) {
        case 1:
		RERR_WRITE32(priv->target, &regs->sfa2ad,
			     priv->amba_base[0] | amba_size_per_chip);
		RERR_WRITE32(priv->target, &regs->sfb1ad,
			     priv->amba_base[0] | amba_size_per_chip);
		RERR_WRITE32(priv->target, &regs->sfb2ad,
			     priv->amba_base[0] | amba_size_per_chip);

                break;
        case 2:
                RERR_WRITE32(priv->target, &regs->sfa2ad,
                             priv->amba_base[1]);
                RERR_WRITE32(priv->target, &regs->sfb1ad,
                             priv->amba_base[1] + amba_size_per_chip);
                RERR_WRITE32(priv->target, &regs->sfb2ad,
                             priv->amba_base[1] + amba_size_per_chip);
                break;
        case 4:
                RERR_WRITE32(priv->target, &regs->sfa2ad,
                             priv->amba_base[2]);
                RERR_WRITE32(priv->target, &regs->sfb1ad,
                             priv->amba_base[3]);
                RERR_WRITE32(priv->target, &regs->sfb2ad,
                             priv->amba_base[3] + amba_size_per_chip);
                break;
        default:
                LOG_ERROR("unsupported chipselect number %u!", priv->num_cs);
                qspi_module_disable(priv, 1);
                return ERROR_FAIL;
        }

	retval = qspi_set_lut(priv);
	if(retval != ERROR_OK)
		return retval;

	for(int cs = 0 ; cs < priv->num_cs; ++cs)
	{
		priv->cur_amba_base = priv->amba_base[cs];
		retval = qspi_op_rdid(priv, (uint32_t*)priv->info, sizeof(priv->info));
		if(retval != ERROR_OK)
			return retval;
		LOG_INFO("Flash JEDEC id: 0x%02x 0x%02x 0x%02x 0x%02x",
			priv->info[0], priv->info[1], priv->info[2], priv->info[3]);

	}

	/* Use 4K erase sector size */
	bank->num_sectors = priv->amba_size / 0x1000;
        bank->sectors = calloc(bank->num_sectors, sizeof(struct flash_sector));
        for (unsigned i = 0; i < bank->num_sectors; i++) {
                bank->sectors[i].offset = offset;
                bank->sectors[i].size = 0x1000;
                bank->sectors[i].is_erased = -1;
                bank->sectors[i].is_protected = -1;
                offset += bank->sectors[i].size;
	}

        qspi_module_disable(priv, 0);
	priv->probed = true;
	return ERROR_OK;
}

int fslqspi_auto_probe(struct flash_bank *bank)
{
        struct fslqspi_flash_bank *priv = bank->driver_priv;
        if (priv->probed)
                return ERROR_OK;
        return fslqspi_probe(bank);
}

int fslqspi_protect_check(struct flash_bank *bank)
{
        struct fslqspi_flash_bank *priv = bank->driver_priv;
	int retval;
	uint8_t enabled = 0;

        if (bank->target->state != TARGET_HALTED) {
                LOG_ERROR("Target not halted");
                return ERROR_TARGET_NOT_HALTED;
        }

        if (!priv->probed)
                return ERROR_FLASH_BANK_NOT_PROBED;

	if((retval = qspi_check_protect(priv, &enabled)) != ERROR_OK)
		return retval;

        for (unsigned int i = 0; i < bank->num_sectors; i++) {
                if (enabled)
                        bank->sectors[i].is_protected = 1;
                else
                        bank->sectors[i].is_protected = 0;
        }

	return ERROR_OK;
}

int fslqspi_get_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	return ERROR_OK;
}

/* flash_bank fslqspi <base> <size> <chip_width> <bus_width> <target#> [options]
 * options:
 *   speed_hz - SPI bus speed - default QSPI_MAX_FREQ
 *   reg_base - QSPI controller register space base - default QSPI_BASE_ADDR
 *   num_cs - number of chip selects - default QSPI_NUM_CS
 *   little_endian - data is in little endian order - default is big endian
 */
FLASH_BANK_COMMAND_HANDLER(fslqspi_flash_bank_command)
{
        struct fslqspi_flash_bank *fslqspi_info;

        if (CMD_ARGC < 6)
                return ERROR_COMMAND_SYNTAX_ERROR;

        fslqspi_info = calloc(1, sizeof(struct fslqspi_flash_bank));
        if (!fslqspi_info) {
                LOG_ERROR("No memory for flash bank info");
                return ERROR_FAIL;
        }
        bank->driver_priv = fslqspi_info;

	fslqspi_info->probed = false;
	fslqspi_info->speed_hz = QSPI_MAX_FREQ;
	fslqspi_info->reg_base = QSPI_BASE_ADDR;
	fslqspi_info->amba_base[0] = QSPI_AMBA_BASE_ADDR;
	fslqspi_info->num_cs = QSPI_NUM_CS;
	fslqspi_info->be = true;

        COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], fslqspi_info->amba_base[0]);
        COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], fslqspi_info->amba_size);
	if(fslqspi_info->amba_base[0] == 0)
		fslqspi_info->amba_base[0] = QSPI_AMBA_BASE_ADDR;
	if(fslqspi_info->amba_size == 0)
		fslqspi_info->amba_size = QSPI_AMBA_SIZE;

        for (unsigned i = 6; i < CMD_ARGC; i++) {
                if (strncmp(CMD_ARGV[i], "speed_hz", 8) == 0)
                        sscanf(CMD_ARGV[i], "%d", &fslqspi_info->speed_hz);
                else if (strncmp(CMD_ARGV[i], "reg_base", 8) == 0)
                        sscanf(CMD_ARGV[i], "%p", &fslqspi_info->reg_base);
                else if (strncmp(CMD_ARGV[i], "num_cs", 9) == 0)
                        sscanf(CMD_ARGV[i], "%d", &fslqspi_info->num_cs);
                else if (strcmp(CMD_ARGV[i], "little_endian") == 0)
                        fslqspi_info->be=false;
        }

        return ERROR_OK;
}

const struct flash_driver fslqspi_flash = {
        .name = "fslqspi",
        .flash_bank_command = fslqspi_flash_bank_command,
        .erase = fslqspi_erase,
        .protect = fslqspi_protect,
        .write = fslqspi_write,
        .read = fslqspi_read,
        .probe = fslqspi_probe,
        .auto_probe = fslqspi_auto_probe,
        /* FIXME: access flash at bus_width size */
        .erase_check = default_flash_blank_check,
        .protect_check = fslqspi_protect_check,
        .info = fslqspi_get_info,
        .free_driver_priv = default_flash_free_driver_priv,
};

