/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

/*****************************************************************************
*
* Filename:
* ---------
*   bq24261.h
*
* Project:
* --------
*   Android
*
* Description:
* ------------
*   bq24261 header file
*
* Author:
* -------
*
****************************************************************************/

#ifndef _bq24261_SW_H_
#define _bq24261_SW_H_

#define bq24261_CON0      0x00
#define bq24261_CON1      0x01
#define bq24261_CON2      0x02
#define bq24261_CON3      0x03
#define bq24261_CON4      0x04
#define bq24261_CON5      0x05
#define bq24261_CON6      0x06
#define bq24261_REG_NUM 7


/**********************************************************
  *
  *   [MASK/SHIFT] 
  *
  *********************************************************/
//CON0
#define CON0_TMR_RST_MASK   0x1
#define CON0_TMR_RST_SHIFT  7

#define CON0_EN_BOOST_MASK   0x1
#define CON0_EN_BOOST_SHIFT  6

#define CON0_STAT_MASK   0x3
#define CON0_STAT_SHIFT  4

#define CON0_EN_SHIPMODE_MASK   0x1
#define CON0_EN_SHIPMODE_SHIFT  3

#define CON0_FAULT_MASK   0x7
#define CON0_FAULT_SHIFT  0

//CON1
#define CON1_RESET_MASK   0x1
#define CON1_RESET_SHIFT  7

#define CON1_IN_LIMIT_MASK   0x7
#define CON1_IN_LIMIT_SHIFT  4

#define CON1_EN_STAT_MASK   0x1
#define CON1_EN_STAT_SHIFT  3

#define CON1_TE_MASK   0x1
#define CON1_TE_SHIFT  2

#define CON1_DIS_CE_MASK   0x1
#define CON1_DIS_CE_SHIFT  1

#define CON1_HZ_MODE_MASK   0x1
#define CON1_HZ_MODE_SHIFT  0

//CON2
#define CON2_VBREG_MASK   0x3F
#define CON2_VBREG_SHIFT  2

#define CON2_MOD_FREQ_MASK   0x3
#define CON2_MOD_FREQ_SHIFT  0

//CON3
#define CON3_VENDER_CODE_MASK   0x7
#define CON3_VENDER_CODE_SHIFT  5

#define CON3_PN_MASK   0x3
#define CON3_PN_SHIFT  3

//CON4
#define CON4_ICHRG_MASK   0x1F
#define CON4_ICHRG_SHIFT  3

#define CON4_ITERM_MASK   0x7
#define CON4_ITERM_SHIFT  0

//CON5
#define CON5_MINSYS_STATUS_MASK   0x1
#define CON5_MINSYS_STATUS_SHIFT  7

#define CON5_VINDPM_STATUS_MASK   0x1
#define CON5_VINDPM_STATUS_SHIFT  6

#define CON5_LOW_CHG_MASK   0x1
#define CON5_LOW_CHG_SHIFT  5

#define CON5_DPDM_EN_MASK   0x1
#define CON5_DPDM_EN_SHIFT  4

#define CON5_CD_STATUS_MASK   0x1
#define CON5_CD_STATUS_SHIFT  3

#define CON5_VINDPM_MASK   0x7
#define CON5_VINDPM_SHIFT  0

//CON6
#define CON6_2XTMR_EN_MASK   0x1
#define CON6_2XTMR_EN_SHIFT  7

#define CON6_TMR_MASK   0x3
#define CON6_TMR_SHIFT  5

#define CON6_BOOST_ILIM_MASK   0x1
#define CON6_BOOST_ILIM_SHIFT  4

#define CON6_TS_EN_MASK   0x1
#define CON6_TS_EN_SHIFT  3

#define CON6_TS_FAULT_MASK   0x3
#define CON6_TS_FAULT_SHIFT  1

#define CON6_VINDPM_OFF_MASK   0x1
#define CON6_VINDPM_OFF_SHIFT  0

/**********************************************************
  *
  *   [Extern Function] 
  *
  *********************************************************/
//CON0----------------------------------------------------
extern void bq24261_set_tmr_rst(unsigned int val);
extern void bq24261_set_en_boost(unsigned int val);
extern unsigned int bq24261_get_stat(void);
extern void bq24261_set_en_shipmode(unsigned int val);
extern unsigned int bq24261_get_fault(void);
//CON1----------------------------------------------------
extern void bq24261_set_reset(unsigned int val);
extern void bq24261_set_in_limit(unsigned int val);
extern void bq24261_set_en_stat(unsigned int val);
extern void bq24261_set_te(unsigned int val);
extern void bq24261_set_dis_ce(unsigned int val);
extern void bq24261_set_hz_mode(unsigned int val);
//CON2----------------------------------------------------
extern void bq24261_set_vbreg(unsigned int val);
extern void bq24261_set_mod_freq(unsigned int val);
//CON3----------------------------------------------------
extern unsigned int bq24261_get_vender_code(void);
extern unsigned int bq24261_get_pn(void);
//CON4----------------------------------------------------
extern void bq24261_set_ichg(unsigned int val);
extern void bq24261_set_iterm(unsigned int val);
//CON5----------------------------------------------------
extern unsigned int bq24261_get_minsys_status(void);
extern unsigned int bq24261_get_vindpm_status(void);
extern void bq24261_set_low_chg(unsigned int val);
extern void bq24261_set_dpdm_en(unsigned int val);
extern unsigned int bq24261_get_cd_status(void);
extern void bq24261_set_vindpm(unsigned int val);
//CON6----------------------------------------------------
extern void bq24261_set_2xtmr_en(unsigned int val);
extern void bq24261_set_tmr(unsigned int val);
extern void bq24261_set_boost_ilim(unsigned int val);
extern void bq24261_set_ts_en(unsigned int val);
extern unsigned int bq24261_get_ts_fault(void);
extern void bq24261_set_vindpm_off(unsigned int val);

//---------------------------------------------------------
extern void bq24261_dump_register(void);
extern unsigned int bq24261_reg_config_interface (unsigned char RegNum, unsigned char val);

extern unsigned int bq24261_read_interface (unsigned char RegNum, unsigned char *val, unsigned char MASK, unsigned char SHIFT);
extern unsigned int bq24261_config_interface (unsigned char RegNum, unsigned char val, unsigned char MASK, unsigned char SHIFT);

#endif // _bq24261_SW_H_

