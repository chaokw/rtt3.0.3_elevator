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


#ifndef __APPLIED_LOGIC_INTERFACE_H__
#define __APPLIED_LOGIC_INTERFACE_H__

#include <rtthread.h>
#include "monitor_interface.h"


#define FLASH_TOTAL_FLAT_ADDR 		0x080E0000
#define FLASH_FLAT_DOWN_ADDR	 	0x080C0000
#define FLASH_FLET_BED_HIGH_ADDR	0x080A0000
#define FLASH_FLET_SPEED_ADDR		0x08080000

#define STM32_FLASH_BASE 0x08000000 	
#define ADDR_FLASH_SECTOR_0     ((u32)0x08000000) 	
#define ADDR_FLASH_SECTOR_1     ((u32)0x08004000) 	
#define ADDR_FLASH_SECTOR_2     ((u32)0x08008000) 	
#define ADDR_FLASH_SECTOR_3     ((u32)0x0800C000) 	
#define ADDR_FLASH_SECTOR_4     ((u32)0x08010000) 	
#define ADDR_FLASH_SECTOR_5     ((u32)0x08020000) 	
#define ADDR_FLASH_SECTOR_6     ((u32)0x08040000) 	
#define ADDR_FLASH_SECTOR_7     ((u32)0x08060000) 	
#define ADDR_FLASH_SECTOR_8     ((u32)0x08080000) 	
#define ADDR_FLASH_SECTOR_9     ((u32)0x080A0000) 	
#define ADDR_FLASH_SECTOR_10    ((u32)0x080C0000) 	
#define ADDR_FLASH_SECTOR_11    ((u32)0x080E0000) 	





typedef struct {
	int32_t total_flat;
	int32_t flat_down;
	int32_t flat_bed_high;
	int32_t flat_speed;
}elevator_flash_info_t;

typedef enum {
	DOOR_STATE = 0,

}ele_state_t;



typedef struct {
	ele_state_t cmd;
	fault_info_t fault;
	//...
	
}e_handle_msg_t;


extern uint32_t STMFLASH_ReadWord(u32 faddr);		  	
extern void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite);		
extern void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead);   		

extern void ele_state_mq(ele_state_t state);
extern void applied_logic_thread_entry(void* parameter);

#endif
