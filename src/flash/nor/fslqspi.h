// SPDX-License-Identifier: GPL-2.0-or-later
/***************************************************************************
 *   Based on u-boot file drivers/spi/fsl_qspi.h                           *
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

#ifndef OPENOCD_FLASH_NOR_FSLQSPI_H
#define OPENOCD_FLASH_NOR_FSLQSPI_H

#define QSPI_BASE_ADDR (void*)0x1550000
#define QSPI_AMBA_BASE_ADDR 0x40000000
#define QSPI_AMBA_SIZE 0x4000000
#define QSPI_MAX_FREQ  50000000
#define QSPI_NUM_CS 1

#define SZ_16M				0x01000000

/* CFI Manufacture ID's */
#define MFR_SPANSION      0x01
#define MFR_STMICRO       0x20
#define MFR_MACRONIX      0xc2
#define MFR_SST           0xbf
#define MFR_WINBOND       0xef
#define MFR_ATMEL         0x1f

#define JEDEC_MFR(id)         (id[0])
#define JEDEC_ID(id)          ((id[1]) << 8 | (id[2]))
#define JEDEC_EXT(id)         ((id[3]) << 8 | (id[4]))
#define FSLQSPI_FLASH_MAX_ID_LEN    6


#define QSPI_IPCR_SEQID_SHIFT           24
#define QSPI_IPCR_SEQID_MASK            (0xf << QSPI_IPCR_SEQID_SHIFT)

#define QSPI_MCR_END_CFD_SHIFT          2
#define QSPI_MCR_END_CFD_MASK           (3 << QSPI_MCR_END_CFD_SHIFT)
#define QSPI_MCR_END_CFD_LE             (1 << QSPI_MCR_END_CFD_SHIFT)
#define QSPI_MCR_DDR_EN_SHIFT           7
#define QSPI_MCR_DDR_EN_MASK            (1 << QSPI_MCR_DDR_EN_SHIFT)
#define QSPI_MCR_CLR_RXF_SHIFT          10
#define QSPI_MCR_CLR_RXF_MASK           (1 << QSPI_MCR_CLR_RXF_SHIFT)
#define QSPI_MCR_CLR_TXF_SHIFT          11
#define QSPI_MCR_CLR_TXF_MASK           (1 << QSPI_MCR_CLR_TXF_SHIFT)
#define QSPI_MCR_MDIS_SHIFT             14
#define QSPI_MCR_MDIS_MASK              (1 << QSPI_MCR_MDIS_SHIFT)
#define QSPI_MCR_RESERVED_SHIFT         16
#define QSPI_MCR_RESERVED_MASK          (0xf << QSPI_MCR_RESERVED_SHIFT)

#define QSPI_SMPR_HSENA_SHIFT           0
#define QSPI_SMPR_HSENA_MASK            (1 << QSPI_SMPR_HSENA_SHIFT)
#define QSPI_SMPR_FSPHS_SHIFT           5
#define QSPI_SMPR_FSPHS_MASK            (1 << QSPI_SMPR_FSPHS_SHIFT)
#define QSPI_SMPR_FSDLY_SHIFT           6
#define QSPI_SMPR_FSDLY_MASK            (1 << QSPI_SMPR_FSDLY_SHIFT)
#define QSPI_SMPR_DDRSMP_SHIFT          16
#define QSPI_SMPR_DDRSMP_MASK           (7 << QSPI_SMPR_DDRSMP_SHIFT) 

#define QSPI_RBSR_RDBFL_SHIFT           8
#define QSPI_RBSR_RDBFL_MASK            (0x3f << QSPI_RBSR_RDBFL_SHIFT)

#define QSPI_RBCT_RXBRD_SHIFT           8
#define QSPI_RBCT_RXBRD_USEIPS          (1 << QSPI_RBCT_RXBRD_SHIFT)

#define QSPI_SR_AHB_ACC_SHIFT           2
#define QSPI_SR_AHB_ACC_MASK            (1 << QSPI_SR_AHB_ACC_SHIFT)
#define QSPI_SR_IP_ACC_SHIFT            1
#define QSPI_SR_IP_ACC_MASK             (1 << QSPI_SR_IP_ACC_SHIFT)
#define QSPI_SR_BUSY_SHIFT              0
#define QSPI_SR_BUSY_MASK               (1 << QSPI_SR_BUSY_SHIFT)

#define QSPI_LCKCR_LOCK                 0x1
#define QSPI_LCKCR_UNLOCK               0x2
 
#define LUT_KEY_VALUE                   0x5af05af0

#define OPRND0_SHIFT                    0
#define OPRND0(x)                       ((x) << OPRND0_SHIFT)
#define PAD0_SHIFT                      8
#define PAD0(x)                         ((x) << PAD0_SHIFT)
#define INSTR0_SHIFT                    10
#define INSTR0(x)                       ((x) << INSTR0_SHIFT)
#define OPRND1_SHIFT                    16
#define OPRND1(x)                       ((x) << OPRND1_SHIFT)
#define PAD1_SHIFT                      24
#define PAD1(x)                         ((x) << PAD1_SHIFT)
#define INSTR1_SHIFT                    26
#define INSTR1(x)                       ((x) << INSTR1_SHIFT)

#define LUT_CMD                         1
#define LUT_ADDR                        2
#define LUT_DUMMY                       3
#define LUT_READ                        7
#define LUT_WRITE                       8

#define LUT_PAD1                        0
#define LUT_PAD2                        1
#define LUT_PAD4                        2

#define ADDR24BIT                       0x18
#define ADDR32BIT                       0x20

struct fslqspi_regs {
        uint32_t mcr;
	uint32_t reserved1;
        uint32_t ipcr;
        uint32_t flshcr;
        uint32_t buf0cr;
        uint32_t buf1cr;
        uint32_t buf2cr;
        uint32_t buf3cr;
        uint32_t bfgencr;
        uint32_t soccr;
	uint32_t reserved2[2];
        uint32_t buf0ind;
        uint32_t buf1ind;
        uint32_t buf2ind;
	uint32_t reserved3[49];
        uint32_t sfar;
	uint32_t reserved4;
        uint32_t smpr;
        uint32_t rbsr;
        uint32_t rbct;
	uint32_t reserved5[15];
        uint32_t tbsr;
        uint32_t tbdr;
	uint32_t reserved6;
        uint32_t sr;
        uint32_t fr;
        uint32_t rser;
        uint32_t spndst;
        uint32_t sptrclr;
	uint32_t reserved7[4];
        uint32_t sfa1ad;
        uint32_t sfa2ad;
        uint32_t sfb1ad;
        uint32_t sfb2ad;
	uint32_t reserved8[28];
        uint32_t rbdr[32];
	uint32_t reserved9[32];
        uint32_t lutkey;
        uint32_t lckcr;
	uint32_t reserved10[2];
        uint32_t lut[64];
};


struct fslqspi_flash_bank {
        bool probed;
	bool be;

	uint8_t info[FSLQSPI_FLASH_MAX_ID_LEN];
	uint32_t speed_hz;
	void* reg_base;
	uint32_t amba_base[4];
	uint32_t amba_size;
	int num_cs;

	uint32_t cur_amba_base;
	uint32_t sf_addr;
        uint8_t num_erase_regions;
        uint32_t *erase_region_info;

	struct target* target;
	struct fslqspi_regs* regs;
};

#define ___swab16(x) \
        ((uint16_t)( \
                (((uint16_t)(x) & (uint16_t)0x00ffU) << 8) | \
                (((uint16_t)(x) & (uint16_t)0xff00U) >> 8) ))
#define ___swab32(x) \
        ((uint32_t)( \
                (((uint32_t)(x) & (uint32_t)0x000000ffUL) << 24) | \
                (((uint32_t)(x) & (uint32_t)0x0000ff00UL) <<  8) | \
                (((uint32_t)(x) & (uint32_t)0x00ff0000UL) >>  8) | \
                (((uint32_t)(x) & (uint32_t)0xff000000UL) >> 24) ))
#define ___swab64(x) \
        ((uint64_t)( \
                (uint64_t)(((uint64_t)(x) & (uint64_t)0x00000000000000ffULL) << 56) | \
                (uint64_t)(((uint64_t)(x) & (uint64_t)0x000000000000ff00ULL) << 40) | \
                (uint64_t)(((uint64_t)(x) & (uint64_t)0x0000000000ff0000ULL) << 24) | \
                (uint64_t)(((uint64_t)(x) & (uint64_t)0x00000000ff000000ULL) <<  8) | \
                (uint64_t)(((uint64_t)(x) & (uint64_t)0x000000ff00000000ULL) >>  8) | \
                (uint64_t)(((uint64_t)(x) & (uint64_t)0x0000ff0000000000ULL) >> 24) | \
                (uint64_t)(((uint64_t)(x) & (uint64_t)0x00ff000000000000ULL) >> 40) | \
                (uint64_t)(((uint64_t)(x) & (uint64_t)0xff00000000000000ULL) >> 56) ))

#endif /* OPENOCD_FLASH_NOR_FSLQSPI_H */
