/*
 * leds-2013.h - RGB LED Driver
 *
 * Copyright (C) 2015 New-bund
 * Aka Jiang <aka.jiang@hotmail.com>
 *
 * December 18, 2015 Init
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LEDS_AW2013_H__
#define __LEDS_AW2013_H__

#define Imax			0x01   //0x00=omA,0x01=5mA,0x02=10mA,0x03=15mA,
#define Rise_time		0x02   //0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define Hold_time		0x01   //0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s
#define Fall_time		0x02   //0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define Off_time		0x01   //0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define Delay_time		0x00   //0x00=0s,0x01=0.13s,0x02=0.26s,0x03=0.52s,0x04=1.04s,0x05=2.08s,0x06=4.16s,0x07=8.32s,0x08=16.64s
#define Period_Num		0x00   //0x00=Coutless,0x01=1,0x02=2, ..., 0x0f=15


#define AW2013_REG_RST			0x00
#define AW2013_REG_GC			0x01
#define AW2013_REG_STATUS		0x02

#define AW2013_REG_EN			0x30

#define AW2013_REG_CTR0			0x31
#define AW2013_REG_CTR1			0x32
#define AW2013_REG_CTR2			0x33

#define AW2013_REG_PWM0			0x34
#define AW2013_REG_PWM1			0x35
#define AW2013_REG_PWM2			0x36

#define AW2013_REG_L0_T0		0x37   //RiseTime[6:4] HoldTime[2:0]
#define AW2013_REG_L0_T1		0x38   //FallTime[6:4] OffTime[2:0]
#define AW2013_REG_L0_T2		0x39   //DelayTime[7:4] PeriodNum[3:0]

#define AW2013_REG_L1_T0		0x3A
#define AW2013_REG_L1_T1		0x3B
#define AW2013_REG_L1_T2		0x3C

#define AW2013_REG_L2_T0		0x3D
#define AW2013_REG_L2_T1		0x3E
#define AW2013_REG_L2_T2		0x3F

#endif
