/*
 * File      : xx.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */

#ifndef __SENSOR_INTERFACE_H__
#define __SENSOR_INTERFACE_H__

#include <rtthread.h>
/*****************************************************************************/
/*    DEFINE DECLARATIONS                                                    */
/*****************************************************************************/

/*HEAD*/
#define FLAT_BED_HEAD_H	0x55
#define FLAT_BED_HEAD_L 0xaa
#define FLAT_BED_HEAD_ACK 0x41
/*CMD*/
#define SET_FLAT_BED_TOTAL_FLAT_CMD		0x40
#define CAL_NOW_FLOOR_CMD		    	0x45
#define GET_FLET_BED_INFO_CMD			0x46
#define RETURN_FLAT_BED_INFO_CMD		0x50


/*radar sensor*/
#define RADAR_SENSOR_EXTI_GPIO_PORT 			GPIOD
#define RADAR_SENSOR_EXTI_GPIO_PIN 				GPIO_Pin_8
#define RADAR_SENSOR_EXTI_SOURCE_GPIO_PORT 		EXTI_PortSourceGPIOD
#define RADAR_SENSOR_EXTI_SOURCE_GPIO_PIN 		EXTI_PinSource8
#define RADAR_SENSOR_EXTI_LINE					EXTI_Line8
#define RADAR_SENSOR_NVIC_IRQ_CHANNEL	 	    EXTI9_5_IRQn
#define RADAR_SENSOR_RCC_APB2			        RCC_APB2Periph_SYSCFG
#define RADAR_SENSOR_RCC_AHB1 					RCC_AHB1Periph_GPIOD
#define RADAR_SENSOR_STATE()  GPIO_ReadInputDataBit(RADAR_SENSOR_EXTI_GPIO_PORT, RADAR_SENSOR_EXTI_GPIO_PIN)



/*calibration sensor*/  //chaokw
#define CAL_SENSOR_EXTI_GPIO_PORT 				GPIOD
#define CAL_SENSOR_EXTI_GPIO_PIN 				GPIO_Pin_9
#define CAL_SENSOR_EXTI_SOURCE_GPIO_PORT 		EXTI_PortSourceGPIOD
#define CAL_SENSOR_EXTI_SOURCE_GPIO_PIN 		EXTI_PinSource9
#define CAL_SENSOR_EXTI_LINE					EXTI_Line9
#define CAL_SENSOR_NVIC_IRQ_CHANNEL	 	    	EXTI9_5_IRQn
#define CAL_SENSOR_RCC_APB2			        	RCC_APB2Periph_SYSCFG
#define CAL_SENSOR_RCC_AHB1 					RCC_AHB1Periph_GPIOD
#define CAL_SENSOR_STATE()  GPIO_ReadInputDataBit(CAL_SENSOR_EXTI_GPIO_PORT, CAL_SENSOR_EXTI_GPIO_PIN)


/*door  sensor*/
#define DOOR_SENSOR_EXTI_GPIO_PORT 				GPIOD
#define DOOR_SENSOR_EXTI_GPIO_PIN 				GPIO_Pin_12
#define DOOR_SENSOR_EXTI_SOURCE_GPIO_PORT 		EXTI_PortSourceGPIOD
#define DOOR_SENSOR_EXTI_SOURCE_GPIO_PIN 		EXTI_PinSource12
#define DOOR_SENSOR_EXTI_LINE					EXTI_Line12
#define DOOR_SENSOR_NVIC_IRQ_CHANNEL	 	    EXTI15_10_IRQn
#define DOOR_SENSOR_RCC_APB2			        RCC_APB2Periph_SYSCFG
#define DOOR_SENSOR_RCC_AHB1 					RCC_AHB1Periph_GPIOD
#define DOOR_SENSOR_STATE()  GPIO_ReadInputDataBit(DOOR_SENSOR_EXTI_GPIO_PORT, DOOR_SENSOR_EXTI_GPIO_PIN)



/*flat bed sensor 485 contrl*/
#define FLAT_BED_485_PORT				GPIOD							
#define FLAT_BED_485_CLK				RCC_AHB1Periph_GPIOD
#define FLAT_BED_485_PIN				GPIO_Pin_4
#define flat_bed_gpio_high()			FLAT_BED_485_PORT->ODR |= FLAT_BED_485_PIN							
#define flat_bed_gpio_low()				FLAT_BED_485_PORT->ODR &= (uint32_t)( ~((uint32_t)FLAT_BED_485_PIN ))




extern int get_flat_bed_info(void);
extern void flat_bed_device_write(const uint8_t *buffer, rt_size_t size);
extern void sensor_thread_entry(void* parameter);
extern int flat_bed_cmd_send(uint8_t cmd,uint8_t *data,uint8_t len);

#endif
