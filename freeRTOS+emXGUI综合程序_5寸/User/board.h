#ifndef __BOARD_H__
#define __BOARD_H__

/*
*************************************************************************
*                             包含的头文件
*************************************************************************
*/
/* STM32 固件库头文件 */
#include "stm32f7xx.h"

/* 开发板硬件bsp头文件 */
#include "./led/bsp_led.h" 
#include "./key/bsp_key.h" 
#include "./lcd/bsp_lcd.h"
#include "./sdram/bsp_sdram.h" 
#include "./touch/bsp_i2c_touch.h"
#include "./touch/gt9xx.h"
#include "./usart/bsp_debug_usart.h"
#include "./flash/bsp_qspi_flash.h"
#include "./fonts/fonts.h"
#include "./tim/bsp_basic_tim.h"
#include "./bsp/mpu/bsp_mpu.h" 
#include "./clock/RTC/bsp_rtc.h"
#include "./beep/bsp_beep.h"   
#include "./adc/bsp_adc.h"
#include "./gyro/mpu6050/bsp_mpu_exti.h"
#include "./wm8978/bsp_wm8978.h"  
#include "./camera/bsp_ov5640.h"

/*
*************************************************************************
*                               函数声明
*************************************************************************
*/
	

#endif /* __BOARD_H__ */
