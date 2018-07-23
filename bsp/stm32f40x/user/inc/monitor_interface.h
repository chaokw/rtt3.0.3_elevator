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


#ifndef __MONITOR_INTERFACE_H__
#define __MONITOR_INTERFACE_H__


#include <rtthread.h>



typedef enum {
	NORMAL = 0,
	FAULT,
	FLAT_CLOSE,
	NON_FLAT_CLOSE
}now_status_t;

typedef enum {
	STOP = 0,
	UP_WALK,
	DOWN_WALK
}direction_t;

typedef enum {
	FLAT_BED = 0,
	NON_FLAT_BED,
}floor_status_t;

typedef enum {	
	CLOSE = 0,
	OPEN
}door_status_t;

typedef enum {
	NO_PEOPLE = 0,
	HAS_PEOPLE
}has_people_t;

typedef struct {
	now_status_t now_status;
	direction_t	 direction;
	floor_status_t floor_status;
	float flatspeed;
	door_status_t door_status;
	has_people_t has_people;
	uint8_t power_status;
	int8_t  now_floor;
	uint8_t version;
	
}monitor_info_t;




typedef enum {
	NON_FAULT = 0,			/*没有故障*/
	RACE_TOP,				/*冲顶*/ 
	RACE_DOWN,				/*蹲底*/ 
	SPEED_PASS,				/*超速*/ 
	RUNNING_OPEN,			/*运行中开门*/ 
	OPNE_RUN_CAR,			/*开门走车*/ 
	OUTSIDE_GATE,			/*门区外停梯*/ 
	FLAT_CLOSE_PEOPLE,		/*平层关人*/ 
	NON_FLAT_CLOSE_PEOPLE,	/*非平层关人*/ 
	POWER_CUT,				/*停电*/ 
	SAFETY_BROKE_DOWN,		/*安全回路断路*/ 
	OPEN_FAULT,				/*开门故障*/ 
	CLOSE_FAULT,			/*关门故障*/ 
	LOCK_LOOP_BROKEN,		/*门锁回路断路*/ 
	FLOOR_BUTTON_STICKS,	/*层站按钮粘连*/ 
	ABNORMAL_SPEED			/*电梯速度异常*/ 
}fault_info_t;

typedef struct {
	fault_info_t faultID;
	int8_t faultFloor;
	uint8_t faultDirection;
	float faultSpeed;
}fault_list_t;

#define MONITOR_485_PORT			GPIOA								
#define MONITOR_485_CLK				RCC_AHB1Periph_GPIOA
#define MONITOR_485_PIN				GPIO_Pin_1

#define monitor_gpio_high()			MONITOR_485_PORT->ODR |= MONITOR_485_PIN							
#define monitor_gpio_low()			MONITOR_485_PORT->ODR &= (uint32_t)( ~((uint32_t)MONITOR_485_PIN ))


extern void disappear_info_to_server(fault_info_t faultID);
extern void fault_info_to_server(fault_info_t faultID);
extern void monitor_info_to_server(void);
extern void monitor_device_write(const uint8_t *buffer, rt_size_t size);
extern void monitor_thread_entry(void* parameter);

#endif
