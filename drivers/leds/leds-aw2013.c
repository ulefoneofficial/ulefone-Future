/*
 * leds-2013.c - RGB LED Driver
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/atomic.h>
#include <linux/leds.h>



#define AW2013_NAME "leds-aw2013"



#define AW2013_DEBUG_ON

#ifdef AW2013_DEBUG_ON
    #define AW2013_DEBUG_TAG                  "[Leds-AW2013] "
    #define AW2013_DEBUG_FUN(f)               pr_err(AW2013_DEBUG_TAG"%s\n", __func__)
    #define AW2013_DEBUG_ERR(fmt, args...)    pr_err(AW2013_DEBUG_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
    #define AW2013_DEBUG_LOG(fmt, args...)    pr_err(AW2013_DEBUG_TAG fmt, ##args)
#else
    #define AW2013_DEBUG_TAG
    #define AW2013_DEBUG_FUN(f)
    #define AW2013_DEBUG_ERR(fmt, args...)
    #define AW2013_DEBUG_LOG(fmt, args...)
#endif

#define AW2013_ID		0x33  // read AW2013_REG_RST default 0x33

#define AW2013_MOD_PWM		0x00   //0x00=PWM,0x01=FLINK
#define AW2013_MOD_FLINK	0x01   //0x00=PWM,0x01=FLINK

#define IMAX		0x01   //0x00=0mA,0x01=5mA,0x02=10mA,0x03=15mA
#define RISE_TIME	0x00   //0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define HOLD_TIME	0x00   //0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s
#define FALL_TIME	0x00   //0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define OFF_TIME	0x03   //0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define DELAY_TIME	0x00   //0x00=0s,0x01=0.13s,0x02=0.26s,0x03=0.52s,0x04=1.04s,0x05=2.08s,0x06=4.16s,0x07=8.32s,0x08=16.64s
#define PERIOD_NUM	0x00   //0x00=Coutless,0x01=1,0x02=2, ..., 0x0f=15

#define AW2013_REG_RST		0x00
#define AW2013_REG_GC		0x01
#define AW2013_REG_STATUS	0x02

#define AW2013_REG_EN		0x30

#define AW2013_REG_CTR0		0x31
#define AW2013_REG_CTR1		0x32
#define AW2013_REG_CTR2		0x33

#define AW2013_REG_PWM0		0x34
#define AW2013_REG_PWM1		0x35
#define AW2013_REG_PWM2		0x36

#define AW2013_REG_L0_T0	0x37   //RiseTime[6:4] HoldTime[2:0]
#define AW2013_REG_L0_T1	0x38   //FallTime[6:4] OffTime[2:0]
#define AW2013_REG_L0_T2	0x39   //DelayTime[7:4] PeriodNum[3:0]

#define AW2013_REG_L1_T0	0x3A
#define AW2013_REG_L1_T1	0x3B
#define AW2013_REG_L1_T2	0x3C

#define AW2013_REG_L2_T0	0x3D
#define AW2013_REG_L2_T1	0x3E
#define AW2013_REG_L2_T2	0x3F


#define NORMAL_LEVEL 255
#define NOTIFICATION_LEVEL 128
#define BAT_FULL_LEVEL 64
#define BAT_MEDIUM_LEVEL 32
#define BAT_LOW_LEVEL 16
#define NOCHARGE_BAT_LOW_LEVEL 8

/*---------------------SET BREATH MODE------------------------------------------*/
#define NOTIFICATION_IMAX			0x02   //0x00=0mA,0x01=5mA,0x02=10mA,0x03=15mA
#define NOTIFICATION_RISE_TIME	0x02   //0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define NOTIFICATION_HOLD_TIME	0x02   //0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s
#define NOTIFICATION_FALL_TIME	0x00   //0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define NOTIFICATION_OFF_TIME		0x03   //0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define NOTIFICATION_DELAY_TIME	0x00   //0x00=0s,0x01=0.13s,0x02=0.26s,0x03=0.52s,0x04=1.04s,0x05=2.08s,0x06=4.16s,0x07=8.32s,0x08=16.64s
#define NOTIFICATION_PERIOD_NUM	0x00   //0x00=Coutless,0x01=1,0x02=2, ..., 0x0f=15


#define BAT_MEDIUM_IMAX		0x01   //0x00=0mA,0x01=5mA,0x02=10mA,0x03=15mA
#define BAT_MEDIUM_RISE_TIME	0x02   //0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define BAT_MEDIUM_HOLD_TIME	0x02   //0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s
#define BAT_MEDIUM_FALL_TIME	0x02   //0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define BAT_MEDIUM_OFF_TIME	0x03   //0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define BAT_MEDIUM_DELAY_TIME	0x01   //0x00=0s,0x01=0.13s,0x02=0.26s,0x03=0.52s,0x04=1.04s,0x05=2.08s,0x06=4.16s,0x07=8.32s,0x08=16.64s
#define BAT_MEDIUM_PERIOD_NUM	0x00   //0x00=Coutless,0x01=1,0x02=2, ..., 0x0f=15

#define BAT_LOW_IMAX			0x01   //0x00=0mA,0x01=5mA,0x02=10mA,0x03=15mA
#define BAT_LOW_RISE_TIME		0x01   //0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define BAT_LOW_HOLD_TIME		0x01   //0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s
#define BAT_LOW_FALL_TIME		0x01   //0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define BAT_LOW_OFF_TIME		0x03   //0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define BAT_LOW_DELAY_TIME	0x03   //0x00=0s,0x01=0.13s,0x02=0.26s,0x03=0.52s,0x04=1.04s,0x05=2.08s,0x06=4.16s,0x07=8.32s,0x08=16.64s
#define BAT_LOW_PERIOD_NUM	0x00   //0x00=Coutless,0x01=1,0x02=2, ..., 0x0f=15

/*---------------------------------------------------------------*/



static DEFINE_MUTEX(aw2013_mutex);

struct aw2013_led_data {
	int r_index;
	int g_index;
	int b_index;
	int r_value;
	int g_value;
	int b_value;
	struct work_struct		work;
	struct i2c_client		*client;
	struct led_classdev		cdev_r;
	struct led_classdev		cdev_g;
	struct led_classdev		cdev_b;

};

static int aw2013_i2c_read_byte(struct i2c_client *client, u8 addr, u8 *data)
{
	u8 beg = addr;
	int err;
	struct i2c_msg msgs[2] = { {0}, {0} };

	mutex_lock(&aw2013_mutex);

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &beg;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = data;

	if (!client) {
		mutex_unlock(&aw2013_mutex);
		return -EINVAL;
	}

	err = i2c_transfer(client->adapter, msgs, sizeof(msgs) / sizeof(msgs[0]));
	if (err != 2) {
		AW2013_DEBUG_ERR("i2c_transfer error: (%d %p) %d\n", addr, data, err);
		err = -EIO;
	} else {
		err = 0;
	}
	mutex_unlock(&aw2013_mutex);
	return err;

}


static int aw2013_i2c_write_byte(struct i2c_client *client, u8 addr, u8 data)
{
	int err;
	char buf[2];

	err = 0;
	mutex_lock(&aw2013_mutex);
	if (!client) {
		mutex_unlock(&aw2013_mutex);
		return -EINVAL;
	} 

	buf[0] = addr;
	buf[1] = data;

	err = i2c_master_send(client, buf, 2);
	if (err < 0) {
		AW2013_DEBUG_ERR("send command error!!\n");
		mutex_unlock(&aw2013_mutex);
		return -EFAULT;
	}
	mutex_unlock(&aw2013_mutex);
	return err;
}

static int aw2013_read_id(struct i2c_client *client)
{
    int ret = 0;
    u8 id = 0;

    ret = aw2013_i2c_read_byte(client, 0x00, &id);
    if(ret < 0){
        return ret;
    }

    AW2013_DEBUG_LOG(" id = 0x%x!", id);
    if(AW2013_ID != id){
        AW2013_DEBUG_LOG(" read id = 0x%x, error!", id);
        return -1;
    }

    return 0;
}

static int aw2013_reg_init(struct i2c_client *client)
{
    int ret = 0;
    ret = aw2013_i2c_write_byte(client, AW2013_REG_RST, 0x55);
    ret = aw2013_i2c_write_byte(client, AW2013_REG_GC,  0x01);
    ret = aw2013_i2c_write_byte(client, AW2013_REG_EN,  0x07);

    ret = aw2013_i2c_write_byte(client, AW2013_REG_CTR0, IMAX);	
    ret = aw2013_i2c_write_byte(client, AW2013_REG_CTR1, IMAX);	
    ret = aw2013_i2c_write_byte(client, AW2013_REG_CTR2, IMAX);

#if 0
    ret = aw2013_i2c_write_byte(client, AW2013_REG_CTR0, IMAX|0x70);	
    ret = aw2013_i2c_write_byte(client, AW2013_REG_CTR1, IMAX|0x70);	
    ret = aw2013_i2c_write_byte(client, AW2013_REG_CTR2, IMAX|0x70);
#endif		

#if 0
    ret = aw2013_i2c_write_byte(client, AW2013_REG_PWM0, 0xFF);
    ret = aw2013_i2c_write_byte(client, AW2013_REG_PWM1, 0xFF);
    ret = aw2013_i2c_write_byte(client, AW2013_REG_PWM2, 0xFF);

    ret = aw2013_i2c_write_byte(client, AW2013_REG_L0_T0, 
				RISE_TIME<<4 | HOLD_TIME);							
    ret = aw2013_i2c_write_byte(client, AW2013_REG_L0_T1, FALL_TIME<<4 | OFF_TIME);	
    ret = aw2013_i2c_write_byte(client, AW2013_REG_L0_T2, DELAY_TIME<<4| PERIOD_NUM);  

    ret = aw2013_i2c_write_byte(client, AW2013_REG_L1_T0, 
				RISE_TIME<<4 | HOLD_TIME);							
    ret = aw2013_i2c_write_byte(client, AW2013_REG_L1_T1, FALL_TIME<<4 | OFF_TIME);	       
    ret = aw2013_i2c_write_byte(client, AW2013_REG_L1_T2, DELAY_TIME<<4| PERIOD_NUM);  

    ret = aw2013_i2c_write_byte(client, AW2013_REG_L2_T0, 
				RISE_TIME<<4 | HOLD_TIME);					
    ret = aw2013_i2c_write_byte(client, AW2013_REG_L2_T1, FALL_TIME<<4 | OFF_TIME);
    ret = aw2013_i2c_write_byte(client, AW2013_REG_L2_T2, DELAY_TIME<<4| PERIOD_NUM);
#endif
    return ret;
}

static int aw2013_leds_all_off(struct aw2013_led_data *pleddata)
{
    int ret = 0;
    struct i2c_client *client = pleddata->client;

    ret = aw2013_i2c_write_byte(client, 0x00, 0x55);

    return ret;
}

static int aw2013_leds_breath(struct aw2013_led_data *pleddata)
{
    int ret = 0;
    int led_en = 0;
    struct i2c_client *client = pleddata->client;
	
	if((pleddata->r_value == NORMAL_LEVEL)||(pleddata->g_value ==NORMAL_LEVEL)||(pleddata->b_value ==NORMAL_LEVEL)){
		
		 ret = aw2013_i2c_write_byte(client, 0x00, 0x55);
	    ret = aw2013_i2c_write_byte(client, 0x01, 0x01);

	    ret = aw2013_i2c_write_byte(client, 0x31, IMAX);
	    ret = aw2013_i2c_write_byte(client, 0x32, IMAX);
	    ret = aw2013_i2c_write_byte(client, 0x33, IMAX);

	    ret = aw2013_i2c_write_byte(client, 0x34 + pleddata->r_index, (pleddata->r_value)|0xff);
	    ret = aw2013_i2c_write_byte(client, 0x34 + pleddata->g_index, (pleddata->g_value)|0xff);
	    ret = aw2013_i2c_write_byte(client, 0x34 + pleddata->b_index, (pleddata->b_value)|0xff);
	}
	else if((pleddata->r_value ==NOTIFICATION_LEVEL)||(pleddata->g_value ==NOTIFICATION_LEVEL)||(pleddata->b_value ==NOTIFICATION_LEVEL)){
		
		ret = aw2013_i2c_write_byte(client, 0x00, 0x55);
	    ret = aw2013_i2c_write_byte(client, 0x01, 0x01);

	    ret = aw2013_i2c_write_byte(client, 0x31, NOTIFICATION_IMAX|0x70);
	    ret = aw2013_i2c_write_byte(client, 0x32, NOTIFICATION_IMAX|0x70);
	    ret = aw2013_i2c_write_byte(client, 0x33, NOTIFICATION_IMAX|0x70);

	    ret = aw2013_i2c_write_byte(client, 0x34 + pleddata->r_index, (pleddata->r_value)|0xff);
	    ret = aw2013_i2c_write_byte(client, 0x34 + pleddata->g_index, (pleddata->g_value)|0xff);
	    ret = aw2013_i2c_write_byte(client, 0x34 + pleddata->b_index, (pleddata->b_value)|0xff);

	    ret = aw2013_i2c_write_byte(client, 0x37, NOTIFICATION_RISE_TIME<<4 | NOTIFICATION_HOLD_TIME);							
	    ret = aw2013_i2c_write_byte(client, 0x38, NOTIFICATION_FALL_TIME<<4 | NOTIFICATION_OFF_TIME);	
	    ret = aw2013_i2c_write_byte(client, 0x39, NOTIFICATION_DELAY_TIME<<4| NOTIFICATION_PERIOD_NUM);  

	    ret = aw2013_i2c_write_byte(client, 0x3A, NOTIFICATION_RISE_TIME<<4 | NOTIFICATION_HOLD_TIME);							
	    ret = aw2013_i2c_write_byte(client, 0x3B, NOTIFICATION_FALL_TIME<<4 | NOTIFICATION_OFF_TIME);	       
	    ret = aw2013_i2c_write_byte(client, 0x3C, NOTIFICATION_DELAY_TIME<<4| NOTIFICATION_PERIOD_NUM);  

	    ret = aw2013_i2c_write_byte(client, 0x3D, NOTIFICATION_RISE_TIME<<4 | NOTIFICATION_HOLD_TIME);					
	    ret = aw2013_i2c_write_byte(client, 0x3E, NOTIFICATION_FALL_TIME<<4 | NOTIFICATION_OFF_TIME);
	    ret = aw2013_i2c_write_byte(client, 0x3F, NOTIFICATION_DELAY_TIME<<4| NOTIFICATION_PERIOD_NUM);
	}
	else if((pleddata->r_value ==BAT_FULL_LEVEL)||(pleddata->g_value ==BAT_FULL_LEVEL)||(pleddata->b_value ==BAT_FULL_LEVEL)){
		
		ret = aw2013_i2c_write_byte(client, 0x00, 0x55);
	    ret = aw2013_i2c_write_byte(client, 0x01, 0x01);

	    ret = aw2013_i2c_write_byte(client, 0x31, IMAX);
	    ret = aw2013_i2c_write_byte(client, 0x32, IMAX);
	    ret = aw2013_i2c_write_byte(client, 0x33, IMAX);

	    ret = aw2013_i2c_write_byte(client, 0x34 + pleddata->r_index, (pleddata->r_value)|0xff);
	    ret = aw2013_i2c_write_byte(client, 0x34 + pleddata->g_index, (pleddata->g_value)|0xff);
	    ret = aw2013_i2c_write_byte(client, 0x34 + pleddata->b_index, (pleddata->b_value)|0xff);
	}
	else if((pleddata->r_value ==BAT_MEDIUM_LEVEL)||(pleddata->g_value ==BAT_MEDIUM_LEVEL)||(pleddata->b_value ==BAT_MEDIUM_LEVEL)){
		
		ret = aw2013_i2c_write_byte(client, 0x00, 0x55);
	    ret = aw2013_i2c_write_byte(client, 0x01, 0x01);

	    ret = aw2013_i2c_write_byte(client, 0x31, BAT_MEDIUM_IMAX|0x70);
	    ret = aw2013_i2c_write_byte(client, 0x32, BAT_MEDIUM_IMAX|0x70);
	    ret = aw2013_i2c_write_byte(client, 0x33, BAT_MEDIUM_IMAX|0x70);

	    ret = aw2013_i2c_write_byte(client, 0x34 + pleddata->r_index, (pleddata->r_value)|0xff);
	    ret = aw2013_i2c_write_byte(client, 0x34 + pleddata->g_index, (pleddata->g_value)|0xff);
	    ret = aw2013_i2c_write_byte(client, 0x34 + pleddata->b_index, (pleddata->b_value)|0xff);

	    ret = aw2013_i2c_write_byte(client, 0x37, BAT_MEDIUM_RISE_TIME<<4 | BAT_MEDIUM_HOLD_TIME);							
	    ret = aw2013_i2c_write_byte(client, 0x38, BAT_MEDIUM_FALL_TIME<<4 | BAT_MEDIUM_OFF_TIME);	
	    ret = aw2013_i2c_write_byte(client, 0x39, BAT_MEDIUM_DELAY_TIME<<4| BAT_MEDIUM_PERIOD_NUM);  

	    ret = aw2013_i2c_write_byte(client, 0x3A, BAT_MEDIUM_RISE_TIME<<4 | BAT_MEDIUM_HOLD_TIME);							
	    ret = aw2013_i2c_write_byte(client, 0x3B, BAT_MEDIUM_FALL_TIME<<4 | BAT_MEDIUM_OFF_TIME);	       
	    ret = aw2013_i2c_write_byte(client, 0x3C, BAT_MEDIUM_DELAY_TIME<<4| BAT_MEDIUM_PERIOD_NUM);  

	    ret = aw2013_i2c_write_byte(client, 0x3D, BAT_MEDIUM_RISE_TIME<<4 | BAT_MEDIUM_HOLD_TIME);					
	    ret = aw2013_i2c_write_byte(client, 0x3E, BAT_MEDIUM_FALL_TIME<<4 | BAT_MEDIUM_OFF_TIME);
	    ret = aw2013_i2c_write_byte(client, 0x3F, BAT_MEDIUM_DELAY_TIME<<4| BAT_MEDIUM_PERIOD_NUM);
	}
	else if((pleddata->r_value ==BAT_LOW_LEVEL)||(pleddata->g_value ==BAT_LOW_LEVEL)||(pleddata->b_value ==BAT_LOW_LEVEL)){
		
		ret = aw2013_i2c_write_byte(client, 0x00, 0x55);
	    ret = aw2013_i2c_write_byte(client, 0x01, 0x01);

	    ret = aw2013_i2c_write_byte(client, 0x31, BAT_LOW_IMAX|0x70);
	    ret = aw2013_i2c_write_byte(client, 0x32, BAT_LOW_IMAX|0x70);
	    ret = aw2013_i2c_write_byte(client, 0x33, BAT_LOW_IMAX|0x70);

	    ret = aw2013_i2c_write_byte(client, 0x34 + pleddata->r_index, (pleddata->r_value)|0xff);
	    ret = aw2013_i2c_write_byte(client, 0x34 + pleddata->g_index, (pleddata->g_value)|0xff);
	    ret = aw2013_i2c_write_byte(client, 0x34 + pleddata->b_index, (pleddata->b_value)|0xff);

	    ret = aw2013_i2c_write_byte(client, 0x37, BAT_LOW_RISE_TIME<<4 | BAT_LOW_HOLD_TIME);							
	    ret = aw2013_i2c_write_byte(client, 0x38, BAT_LOW_FALL_TIME<<4 | BAT_LOW_OFF_TIME);	
	    ret = aw2013_i2c_write_byte(client, 0x39, BAT_LOW_DELAY_TIME<<4| BAT_LOW_PERIOD_NUM);  

	    ret = aw2013_i2c_write_byte(client, 0x3A, BAT_LOW_RISE_TIME<<4 | BAT_LOW_HOLD_TIME);							
	    ret = aw2013_i2c_write_byte(client, 0x3B, BAT_LOW_FALL_TIME<<4 | BAT_LOW_OFF_TIME);	       
	    ret = aw2013_i2c_write_byte(client, 0x3C, BAT_LOW_DELAY_TIME<<4| BAT_LOW_PERIOD_NUM);  

	    ret = aw2013_i2c_write_byte(client, 0x3D, BAT_LOW_RISE_TIME<<4 | BAT_LOW_HOLD_TIME);					
	    ret = aw2013_i2c_write_byte(client, 0x3E, BAT_LOW_FALL_TIME<<4 | BAT_LOW_OFF_TIME);
	    ret = aw2013_i2c_write_byte(client, 0x3F, BAT_LOW_DELAY_TIME<<4| BAT_LOW_PERIOD_NUM);
	}
	else if((pleddata->r_value ==NOCHARGE_BAT_LOW_LEVEL)||(pleddata->g_value ==NOCHARGE_BAT_LOW_LEVEL)||(pleddata->b_value ==NOCHARGE_BAT_LOW_LEVEL)){
		
		ret = aw2013_i2c_write_byte(client, 0x00, 0x55);
	    ret = aw2013_i2c_write_byte(client, 0x01, 0x01);

	    ret = aw2013_i2c_write_byte(client, 0x31, IMAX|0x10);
	    ret = aw2013_i2c_write_byte(client, 0x32, IMAX|0x10);
	    ret = aw2013_i2c_write_byte(client, 0x33, IMAX|0x10);

	    ret = aw2013_i2c_write_byte(client, 0x34 + pleddata->r_index, pleddata->r_value|0xff);
	    ret = aw2013_i2c_write_byte(client, 0x34 + pleddata->g_index, pleddata->g_value|0xff);
	    ret = aw2013_i2c_write_byte(client, 0x34 + pleddata->b_index, pleddata->b_value|0xff);

	    ret = aw2013_i2c_write_byte(client, 0x37, RISE_TIME<<4 | HOLD_TIME);							
	    ret = aw2013_i2c_write_byte(client, 0x38, FALL_TIME<<4 | OFF_TIME);	
	    ret = aw2013_i2c_write_byte(client, 0x39, DELAY_TIME<<4| PERIOD_NUM);  

	    ret = aw2013_i2c_write_byte(client, 0x3A, RISE_TIME<<4 | HOLD_TIME);							
	    ret = aw2013_i2c_write_byte(client, 0x3B, FALL_TIME<<4 | OFF_TIME);	       
	    ret = aw2013_i2c_write_byte(client, 0x3C, DELAY_TIME<<4| PERIOD_NUM);  

	    ret = aw2013_i2c_write_byte(client, 0x3D, RISE_TIME<<4 | HOLD_TIME);					
	    ret = aw2013_i2c_write_byte(client, 0x3E, FALL_TIME<<4 | OFF_TIME);
	    ret = aw2013_i2c_write_byte(client, 0x3F, DELAY_TIME<<4| PERIOD_NUM);
	}
	else{
		ret = aw2013_i2c_write_byte(client, 0x00, 0x55);
		ret = aw2013_i2c_write_byte(client, 0x01, 0x01);
	}

    if(pleddata->r_value){
        led_en |= 1 << pleddata->r_index;
    }

    if(pleddata->g_value){
        led_en |= 1 << pleddata->g_index;
    }

    if(pleddata->b_value){
        led_en |= 1 << pleddata->b_index;
    }

    ret = aw2013_i2c_write_byte(client, 0x30, led_en); 

    return ret;
}

static int aw2013_leds_flash(struct aw2013_led_data *pleddata)
{
    int ret = 0;
    int led_en = 0;
    struct i2c_client *client = pleddata->client;

    ret = aw2013_i2c_write_byte(client, 0x00, 0x55);
    ret = aw2013_i2c_write_byte(client, 0x01, 0x01);

    ret = aw2013_i2c_write_byte(client, 0x31, IMAX|0x10);
    ret = aw2013_i2c_write_byte(client, 0x32, IMAX|0x10);
    ret = aw2013_i2c_write_byte(client, 0x33, IMAX|0x10);

    ret = aw2013_i2c_write_byte(client, 0x34 + pleddata->r_index, pleddata->r_value);
    ret = aw2013_i2c_write_byte(client, 0x34 + pleddata->g_index, pleddata->g_value);
    ret = aw2013_i2c_write_byte(client, 0x34 + pleddata->b_index, pleddata->b_value);

    ret = aw2013_i2c_write_byte(client, 0x37, RISE_TIME<<4 | HOLD_TIME);							
    ret = aw2013_i2c_write_byte(client, 0x38, FALL_TIME<<4 | OFF_TIME);	
    ret = aw2013_i2c_write_byte(client, 0x39, DELAY_TIME<<4| PERIOD_NUM);  

    ret = aw2013_i2c_write_byte(client, 0x3A, RISE_TIME<<4 | HOLD_TIME);							
    ret = aw2013_i2c_write_byte(client, 0x3B, FALL_TIME<<4 | OFF_TIME);	       
    ret = aw2013_i2c_write_byte(client, 0x3C, DELAY_TIME<<4| PERIOD_NUM);  

    ret = aw2013_i2c_write_byte(client, 0x3D, RISE_TIME<<4 | HOLD_TIME);					
    ret = aw2013_i2c_write_byte(client, 0x3E, FALL_TIME<<4 | OFF_TIME);
    ret = aw2013_i2c_write_byte(client, 0x3F, DELAY_TIME<<4| PERIOD_NUM);

    if(pleddata->r_value){
        led_en |= 1 << pleddata->r_index;
    }

    if(pleddata->g_value){
        led_en |= 1 << pleddata->g_index;
    }

    if(pleddata->b_value){
        led_en |= 1 << pleddata->b_index;
    }

    ret = aw2013_i2c_write_byte(client, 0x30, led_en); 

    return ret;
}

static int aw2013_leds_onoff(struct aw2013_led_data *pleddata)
{
    int ret = 0;
    int led_en = 0;
    struct i2c_client *client = pleddata->client;

    ret = aw2013_i2c_write_byte(client, 0x00, 0x55);
    ret = aw2013_i2c_write_byte(client, 0x01, 0x01);

    ret = aw2013_i2c_write_byte(client, 0x31, IMAX);
    ret = aw2013_i2c_write_byte(client, 0x32, IMAX);
    ret = aw2013_i2c_write_byte(client, 0x33, IMAX);

    ret = aw2013_i2c_write_byte(client, 0x34 + pleddata->r_index, pleddata->r_value);
    ret = aw2013_i2c_write_byte(client, 0x34 + pleddata->g_index, pleddata->g_value);
    ret = aw2013_i2c_write_byte(client, 0x34 + pleddata->b_index, pleddata->b_value);

    if(pleddata->r_value){
        led_en |= 1 << pleddata->r_index;
    }

    if(pleddata->g_value){
        led_en |= 1 << pleddata->g_index;
    }

    if(pleddata->b_value){
        led_en |= 1 << pleddata->b_index;
    }

    ret = aw2013_i2c_write_byte(client, 0x30, led_en); 

    return ret;
}


static void aw2013_work(struct work_struct *work)
{
    struct aw2013_led_data *pleddata = container_of(work, struct aw2013_led_data, work);
    struct i2c_client *client = pleddata->client;
    int ret = 0;

    if((pleddata->r_value == 0) && (pleddata->g_value == 0) && (pleddata->b_value == 0)){
        ret = aw2013_leds_all_off(pleddata);
    }else{
        //ret = aw2013_leds_onoff(pleddata);
        //ret = aw2013_leds_flash(pleddata);
		ret = aw2013_leds_breath(pleddata);
    }
    
}


static void aw2013_set_r_brightness(struct led_classdev *led_cdev, enum led_brightness value)
{
    struct aw2013_led_data *pleddata = container_of(led_cdev, struct aw2013_led_data, cdev_r);
    pleddata->r_value = value & 0xFF;
    schedule_work(&pleddata->work);
}

static void aw2013_set_g_brightness(struct led_classdev *led_cdev, enum led_brightness value)
{
    struct aw2013_led_data *pleddata = container_of(led_cdev, struct aw2013_led_data, cdev_g);
    pleddata->g_value = value & 0xFF;
    schedule_work(&pleddata->work);
}

static void aw2013_set_b_brightness(struct led_classdev *led_cdev, enum led_brightness value)
{
    struct aw2013_led_data *pleddata = container_of(led_cdev, struct aw2013_led_data, cdev_b);
    pleddata->b_value = value & 0xFF;
    schedule_work(&pleddata->work);
}

static int aw2013_register_led_classdev(struct aw2013_led_data *pleddata)
{
    int ret;

    INIT_WORK(&pleddata->work, aw2013_work);

    pleddata->r_value = LED_OFF;
    pleddata->cdev_r.name = "red";
    pleddata->cdev_r.brightness = LED_OFF;
    pleddata->cdev_r.brightness_set = aw2013_set_r_brightness;
    //pleddata->cdev_r.blink_set = ;

    ret = led_classdev_register(&pleddata->client->dev, &pleddata->cdev_r);
    if (ret < 0) {
        AW2013_DEBUG_ERR("couldn't register LED %s\n", pleddata->cdev_r.name);
        goto failed_unregister_led_r;
    }

    pleddata->g_value = LED_OFF;
    pleddata->cdev_g.name = "green";
    pleddata->cdev_g.brightness = LED_OFF;
    pleddata->cdev_g.brightness_set = aw2013_set_g_brightness;
    //pleddata->cdev_g.blink_set = ;

    ret = led_classdev_register(&pleddata->client->dev, &pleddata->cdev_g);
    if (ret < 0) {
        AW2013_DEBUG_ERR("couldn't register LED %s\n", pleddata->cdev_g.name);
        goto failed_unregister_led_g;
    }

    pleddata->b_value = LED_OFF;
    pleddata->cdev_b.name = "blue";
    pleddata->cdev_b.brightness = LED_OFF;
    pleddata->cdev_b.brightness_set = aw2013_set_b_brightness;
    //pleddata->cdev_b.blink_set = ;

    ret = led_classdev_register(&pleddata->client->dev, &pleddata->cdev_b);
    if (ret < 0) {
        AW2013_DEBUG_ERR("couldn't register LED %s\n", pleddata->cdev_b.name);
        goto failed_unregister_led_g;
    }

    AW2013_DEBUG_LOG("OK!");
    return 0;

failed_unregister_led_b:
    led_classdev_unregister(&pleddata->cdev_g);
failed_unregister_led_g:
    led_classdev_unregister(&pleddata->cdev_r);
failed_unregister_led_r:
    AW2013_DEBUG_LOG("Fail!");
    return ret;
}

static void aw2013_unregister_led_classdev(struct aw2013_led_data *pleddata)
{
    cancel_work_sync(&pleddata->work);
    led_classdev_unregister(&pleddata->cdev_r);
    led_classdev_unregister(&pleddata->cdev_g);
    led_classdev_unregister(&pleddata->cdev_b);
}



// led0   blue
// led1   green
// led2   red
static int aw2013_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    
    int ret = 0;
    struct aw2013_led_data *pleddata;

    AW2013_DEBUG_LOG("Enter!");

    pleddata = devm_kzalloc(&client->dev, sizeof(struct aw2013_led_data), GFP_KERNEL);
    if (!pleddata) {
        AW2013_DEBUG_ERR("Alloc Memory Error!");
        ret = -ENOMEM;
        goto exit;
    }
    memset(pleddata, 0, sizeof(struct aw2013_led_data));
    pleddata->client = client;
    i2c_set_clientdata(client, pleddata);

    // read from dts
    pleddata->r_index = 2;
    pleddata->g_index = 1;
    pleddata->b_index = 0;

    ret = aw2013_register_led_classdev(pleddata);
    if(ret < 0){
        goto free_mem;
    }

    ret = aw2013_read_id(client);
    if(ret < 0){
        goto free_mem;
    }

#if 1
    ret = aw2013_reg_init(client);
    if(ret < 0){
        goto free_mem;
    }
#endif

    AW2013_DEBUG_LOG("OK!");
    return 0;

free_mem:
    kfree(pleddata);
exit:
    AW2013_DEBUG_LOG("Fail!");
    return ret;
}

static int aw2013_remove(struct i2c_client *client)
{
    struct aw2013_led_data *pleddata = i2c_get_clientdata(client);

    aw2013_unregister_led_classdev(pleddata);
    kfree(pleddata);

    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int aw2013_suspend(struct device *dev)
{
	return 0;
}

static int aw2013_resume(struct device *dev)
{
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(aw2013_pm, aw2013_suspend, aw2013_resume);

static const struct i2c_device_id aw2013_id[] = {
	{ AW2013_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aw2013_id);

static struct i2c_driver aw2013_i2c_driver = {
	.driver	= {
		.name	= AW2013_NAME,
		.pm	= &aw2013_pm,
	},
	.probe		= aw2013_probe,
	.remove		= aw2013_remove,
	.id_table	= aw2013_id,
};

module_i2c_driver(aw2013_i2c_driver);

MODULE_AUTHOR("Aka Jiang <aka.jiang@hotmail.com>");
MODULE_DESCRIPTION("AW2013 LED driver");
MODULE_LICENSE("GPL v2");
