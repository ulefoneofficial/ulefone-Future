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

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>

#define IMX258_PDAFOTP_DEBUG//xielei
#ifdef IMX258_PDAFOTP_DEBUG
#define PFX "IMX258_pdafotp"
#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)
#else
#define LOG_INF(format, args...)
#endif

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern void kdSetI2CSpeed(u16 i2cSpeed);
//extern int iBurstWriteReg_multi(u8 *pData, u32 bytes, u16 i2cId, u16 transfer_length);
extern int iMultiReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId, u8 number);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);


#define USHORT             unsigned short
#define BYTE               unsigned char
#define Sleep(ms) mdelay(ms)

#define IMX258_EEPROM_READ_ID  0xb0
#define IMX258_EEPROM_WRITE_ID   0xb1
#define IMX258_I2C_SPEED        400  //100
#define IMX258_MAX_OFFSET		0xFFFF

#define DATA_SIZE 2048
BYTE imx258_eeprom_data[DATA_SIZE]= {0};
static bool get_done = false;
static int last_size = 0;
static int last_offset = 0;


static bool selective_read_eeprom(kal_uint16 addr, BYTE* data)
{
	char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    if(addr > IMX258_MAX_OFFSET)
        return false;

	kdSetI2CSpeed(IMX258_I2C_SPEED);

	if(iReadRegI2C(pu_send_cmd, 2, (u8*)data, 1, IMX258_EEPROM_READ_ID)<0)
	{
		LOG_INF("read_eeprom imx258 read-id fail\n");
		return false;
	}
		
    return true;
}

static bool _read_imx258_eeprom(kal_uint16 addr, BYTE* data, kal_uint32 size ){
	int i = 0, count = 0;
	//int offset = addr;
	// get PDAF calibration step 1 data
	int offset = 0x0791; 
	for(i = 0; i < 496; i++) {
		if(!selective_read_eeprom(offset, &data[i])){
			LOG_INF("read_eeprom 0x%0x %d fail \n",offset, data[i]);
			return false;
		}
		LOG_INF("read_eeprom 0x%0x    data[%d]0x%x\n",offset, i, data[i]);
		offset++;
		count++;
	}
	
	// get PDAF calibration step 2 data
	offset = 0x0983;
	for(i = 498; i < 1358; i++) {
		if(!selective_read_eeprom(offset, &data[i])){
			LOG_INF("read_eeprom 0x%0x %d fail \n",offset, data[i]);
			return false;
		}
		LOG_INF("read_eeprom 0x%0x    data[%d]0x%x\n",offset, i, data[i]);
		offset++;
		count++;
	}
	
  /* for(i=1358;i<2048;i++){
       data[i]=0x00; 
   }
	*/
	LOG_INF("[mcnex]read_eeprom data count = %d\n",count);
	get_done = true;
	last_size = size;
	last_offset = addr;
    return true;
}

bool read_imx258_eeprom( kal_uint16 addr, BYTE* data, kal_uint32 size){
	//addr = 0x0763;
	//size = 1404;

	LOG_INF("read imx258 eeprom, size = %d\n", size);

	if (1)//(!get_done || last_size != size || last_offset != addr) {
		{
		if(!_read_imx258_eeprom(addr, imx258_eeprom_data, size)){
			LOG_INF("read_imx258_eeprom failed\n");
			get_done = 0;
            last_size = 0;
            last_offset = 0;
			return false;
		}
	}
	//memcpy(data, imx258_eeprom_data, size);
    return true;
}

bool read_imx258_eeprom_SPC( kal_uint16 addr, BYTE* data, kal_uint32 size){

	//addr = 0x0F73;//0x0F73;
	//size = 126;

	LOG_INF("read imx258 eeprom, size = %d\n", size);

	//if(!get_done || last_size != size || last_offset != addr) {
		if(!_read_imx258_eeprom(addr, imx258_eeprom_data, size)){
			LOG_INF("read_imx258_eeprom_SPC failed\n");
			get_done = 0;
            last_size = 0;
            last_offset = 0;
			return false;
		}
	//}

	memcpy(data, imx258_eeprom_data, size);
    return true;
}
static bool selective_write_eeprom(kal_uint16 addr,kal_uint8 para)
{
    //kal_uint16 get_byte=0;
	//char pu_send_cmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
    if(addr > IMX258_MAX_OFFSET)
        return false;
	kdSetI2CSpeed(IMX258_I2C_SPEED);
	//LOG_INF("write_eeprom 0x%0x 0x%0x 0x%0x\n",pu_send_cmd[0], pu_send_cmd[1],pu_send_cmd[2]);
	//if(iWriteRegI2C(pu_send_cmd , 3, IMX258_EEPROM_WRITE_ID)<0){
	if(iWriteReg(addr,para,1, IMX258_EEPROM_WRITE_ID)<0){
		LOG_INF("write_eeprom imx258 write-id fail\n");
		return false;		
	}
    return true;
}
bool _write_imx258_eeprom(kal_uint16 addr, kal_uint8* data, kal_uint32 size ){
	int i = 0;
	int offset = addr;
	for(i = 0; i < size; i++) {
		if(!selective_write_eeprom(offset, data[i])){
			LOG_INF("_write_imx258_eeprom s5k3p3 write-id fail\n");
			return false;
		}
		LOG_INF("write_eeprom 0x%0x 0x%0x\n",offset, data[i]);
		offset++;
	}
	//get_done = true;
	//last_size = size;
	//last_offset = addr;
    return true;
}
bool write_imx258_eeprom( kal_uint16 addr, kal_uint8 data, kal_uint32 size){


	LOG_INF("write 3P3 eeprom, size = %d\n", size);

	//if(!get_done || last_size != size || last_offset != addr) {
		if(!selective_write_eeprom(addr, data)){
			return false;
		}
	//}

    return true;
}

