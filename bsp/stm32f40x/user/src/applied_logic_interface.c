/* File      : xx.c
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


/*****************************************************************************/
/*    INCLUDE FILE DECLARATIONS                                              */
/*****************************************************************************/
#include <rtthread.h>
#include <stm32f4xx.h>
#include <finsh.h>
#include <shell.h>
#include "string.h"
#include "sensor_interface.h"
#include "monitor_interface.h"
#include "applied_logic_interface.h"
#include "stm32f4xx_flash.h"



/*****************************************************************************/
/*    DEFINE DECLARATIONS                                                    */
/*****************************************************************************/
#define HAS_PEOPLE_TIME (10*1)		  		/* 10 s */   //chaokw
#define HAS_PEOPLE_TIME_DELAY (10*20) 		/* 20 s */	
#define CLOSE_PEOPLE_TIME	(10*60*3) 		/* 3 min*/
#define OUT_SIDE_DOOR_TIME (10*60*3)		/* 3 min*/
#define CLOSE_DOOR_FAULT_TIME  (10*60*3)	/* 3 min*/
#define MONITOR_TIME	(10*1)				/*5 s*/
#define FAULT_TIME		(10*10)				/*10 s*/



static struct rt_timer g_dev_timer;
extern monitor_info_t g_monitor_info;
elevator_flash_info_t g_elevator_flash_info;



fault_list_t g_fault_list[16] = {0};


/* 消息队列控制块 */
struct rt_messagequeue g_fault_mq;

/* 消息队列中用到的放置消息的内存池 */
static char g_fault_msg_pool[2048];
e_handle_msg_t g_ele_msg = {0};

/*****************************************************************************/
/*    PRIVATE FUNCTION DECLARATIONS                                          */
/*****************************************************************************/


uint32_t STMFLASH_ReadWord(u32 faddr)
{
	return *(vu32*)faddr; 
}  

uint16_t STMFLASH_GetFlashSector(u32 addr)
{
	if(addr<ADDR_FLASH_SECTOR_1)return FLASH_Sector_0;
	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_Sector_1;
	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_Sector_2;
	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_Sector_3;
	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_Sector_4;
	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_Sector_5;
	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_Sector_6;
	else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_Sector_7;
	else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_Sector_8;
	else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_Sector_9;
	else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_Sector_10; 
	return FLASH_Sector_11;	
}

void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)	
{ 
	FLASH_Status status = FLASH_COMPLETE;
	u32 addrx=0;
	u32 endaddr=0;	
	if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	
	FLASH_Unlock();								
	FLASH_DataCacheCmd(DISABLE);
 		
	addrx=WriteAddr;				
	endaddr=WriteAddr+NumToWrite*4;	
	if(addrx<0X1FFF0000)			
	{
		while(addrx<endaddr)		
		{
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)
			{   
				status=FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);
				if(status!=FLASH_COMPLETE)break;	
			}else addrx+=4;
		} 
	}
	if(status==FLASH_COMPLETE)
	{
		while(WriteAddr<endaddr)
		{
			if(FLASH_ProgramWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE)
			{ 
				break;	
			}
			WriteAddr+=4;
			pBuffer++;
		} 
	}
	FLASH_DataCacheCmd(ENABLE);	
	FLASH_Lock();
} 


void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);
		ReadAddr+=4;
	}
}



static void send_flat_fault_timeout(void* parameter)
{		
	get_flat_bed_info();

}

void flat_fault_timer_init(void)
{
	rt_timer_init(
		&g_dev_timer,
		"elevator_timer",
		send_flat_fault_timeout,
		RT_NULL,
		5,		/*50ms*/
		RT_TIMER_FLAG_PERIODIC
	);
	rt_timer_start(&g_dev_timer);
}

void set_fault_info(fault_info_t fault_info)
{
	g_fault_list[fault_info].faultID = fault_info;
	g_fault_list[fault_info].faultFloor = g_monitor_info.now_floor;
	g_fault_list[fault_info].faultDirection = g_monitor_info.direction;

	if ((fault_info == FLAT_CLOSE_PEOPLE) || 
		(fault_info == CLOSE_FAULT) ||
		(fault_info == OPEN_FAULT))
	{
		g_fault_list[fault_info].faultSpeed = 0;

	} else {
		if (g_monitor_info.flatspeed == 0) {
			g_fault_list[fault_info].faultSpeed = 1.05;
		} else {
			g_fault_list[fault_info].faultSpeed = g_monitor_info.flatspeed;
		}

	}
}

/*

{
	monitor:
	参数			类型	说明
	now_status		0-3		当前状态[0 正常 1 故障 2 平层关人 3 非平层关人]		[ok]
	direction		0-2		方向[0 停留 1 上行 2 下行]							[ok]
	floor_status	0-1		平层状态[0平层 1 非平层]							[ok]
	speed			浮点	当时电梯速度										[ok]
	door_status		0-3		门状态[0 关门 1 开门 2 关门中 3 开门中]				[ok]
	has_people		0-1		人状态[0 无人 1 有人]								[ok]
	power_status	0-2		电力[0 电源 1 电池 2 其他]							[ok]
	now_floor		整形	当前楼层											[ok]
	version			整形	扩展参数版本，无扩展则为0							[ok]
	
	*************************************************************************************
	fault:
	编号 说明		
	01   冲顶 				[ok]		【最顶层时上行状态】					
	02   蹲底 				[ok]		【最底层时下行状态】		
	03   超速 				[ok]		【超过电梯额定速度15%，电梯速度串口设定】
	04   运行中开门 		[ok]		【运行状态下突然开门】			
	05   开门走车 			[ok]		【开门状态下运行】		
	06   门区外停梯 		[ok]		【不在平层状态下停车】				
	07   平层关人 			[ok]		【平层状态下关人】		
	08   非平层关人 		[ok]		【非平层状态下关人】				
	09   停电 				[待续]			
	10   安全回路断路 		[暂不实现]					
	11   开门故障 			[ok]		【同一平层状态下开关门6次】			
	12   关门故障 			[ok]		【同一平层下保持开门状态3分钟】			
	13   门锁回路断路 		[暂不实现]					
	14   层站按钮粘连 		[暂不实现]					
	15   电梯速度异常		[ok] 	【超过电梯额定速度15%，电梯速度串口设定，同超速】

	消除故障:【平层状态下,开关门一次！！！】	
}
*/
void monitor_report_handle(void)
{
	uint8_t i;
	uint32_t static outSideCnt = 0;
	uint32_t static timeCnt = 0;
	uint32_t static nonTimeCnt = 0;
	uint32_t static fault_time = 0;
	uint8_t static door_fault_time = 0;
	uint8_t static door_start_time = 0;
	uint32_t static running_open_time = 0;
	uint32_t static door_open_time = 0;
	uint32_t static monitor_time = 0;
	uint32_t static door_close_time = 0;
	floor_status_t static lastFloorStatus = 0; 
	int8_t static lastFloorBed = 0;
	has_people_t  currentHasPeople = 0;
	door_status_t static lastDoorStatus = 0;
	
	uint8_t static speed_start_flat = 0;
	uint8_t static speed_start_flag = 0;
	float static speed_start_time = 0;
	float speed_end_time = 0;
	float flatHigh = 0;
	char speed_info[100] = {0};

	uint32_t static down_walk_time = 0;
	uint32_t static up_walk_time = 0;
	int32_t highFlat = 0;

	uint8_t calFlat = 1;  //chaokw

	/*电梯的基本信息(楼层，停止，运行，上下行)*/
	//get_flat_bed_info();

	/*门和人体雷达的基本信息*/  
	uint32_t static has_people_time = 0;
	uint8_t static has_people_flag = 1;
	currentHasPeople = RADAR_SENSOR_STATE();
	calFlat = CAL_SENSOR_STATE();  //chaokw
	//rt_kprintf("\r\n[CAL_SENSOR_STATE] -> [%d]\r\n",calFlat);
	if (calFlat == 0) {
		uint8_t nowFloor = 0;
		rt_kprintf("\r\ncal now floor to one!!!\r\n");
		flat_bed_cmd_send(CAL_NOW_FLOOR_CMD,(uint8_t *)&nowFloor,2);
	}

	if (currentHasPeople == HAS_PEOPLE) {
		g_monitor_info.has_people = HAS_PEOPLE;
		has_people_flag = 1;
		has_people_time = 0;
	}

	if (has_people_flag == 1) {
		if (has_people_time++ > HAS_PEOPLE_TIME) {
			has_people_flag = 0;
			has_people_time = 0;
			g_monitor_info.has_people = currentHasPeople;
		}
	}

	
	if (g_monitor_info.direction == STOP) {
		g_monitor_info.floor_status = FLAT_BED;
	} else {
		g_monitor_info.floor_status = NON_FLAT_BED;
	}
	
	/*get now status*/
	/*初始正常状态*/
	uint8_t static hasPeopleFlag = 0;
	
	g_monitor_info.now_status = NORMAL;		

	/*平层关人:3分钟判断时间，太短了容易误报,人体雷达设置最短时间检测到人体15秒后清零*/
	if((lastFloorStatus == FLAT_BED) && 
	   (lastDoorStatus == CLOSE) && 
	   (lastFloorBed == g_monitor_info.now_floor))
	{
		timeCnt++;
		/*如果连续3分钟以上四个条件都满足，则平层关人*/

		if (timeCnt > (HAS_PEOPLE_TIME + HAS_PEOPLE_TIME_DELAY)) {
			if (g_monitor_info.has_people == HAS_PEOPLE) {
				hasPeopleFlag = 1;
			}
		}

		if ((timeCnt > CLOSE_PEOPLE_TIME) && (hasPeopleFlag == 1)){
			g_monitor_info.now_status = FLAT_CLOSE;

			set_fault_info(FLAT_CLOSE_PEOPLE);
			rt_kprintf("Flat close people\r\n");
		} 
	} else {
		timeCnt = 0;
		hasPeopleFlag = 0;
	}

	/*非平层关人:3分钟判断时间，太短了容易误报,人体雷达设置最短时间检测到人体15秒后清零*/
	uint8_t static nonHasPeopleFlag = 0;
	
	if((lastFloorStatus == NON_FLAT_BED) && 
	   (lastDoorStatus == CLOSE) && 
	   (lastFloorBed == g_monitor_info.now_floor))
	{
		nonTimeCnt++;
		/*如果连续3分钟以上四个条件都满足，则非平层关人*/
		if (nonTimeCnt > (HAS_PEOPLE_TIME + HAS_PEOPLE_TIME_DELAY)) {
			if(g_monitor_info.has_people == HAS_PEOPLE) {
				nonHasPeopleFlag = 1;
			}
		}
		
		if ((nonTimeCnt > CLOSE_PEOPLE_TIME) && (nonHasPeopleFlag == 1)){
			g_monitor_info.now_status = NON_FLAT_CLOSE;
			
			set_fault_info(NON_FLAT_CLOSE_PEOPLE);
			rt_kprintf("non Flat close people\r\n");
		}
	} else {
		nonTimeCnt = 0;
		nonHasPeopleFlag = 0;
	}

	/*电梯的速度*/
	if (speed_start_flag == 1) {
		speed_start_time++;
		if ((g_monitor_info.floor_status != lastFloorStatus) &&
		   (speed_start_time < 5))
		{
			speed_start_time = 0;
		}
	}
	if (lastFloorBed != g_monitor_info.now_floor) {
		lastFloorStatus = g_monitor_info.floor_status;
		speed_end_time = speed_start_time / 10;

		flatHigh = (float)g_elevator_flash_info.flat_bed_high/100;
		
		if (speed_end_time == 0) {
			g_monitor_info.flatspeed = 0;
		} else {
			g_monitor_info.flatspeed = flatHigh / speed_end_time;
		}
		speed_start_flag = 1;
		speed_start_time = 0;
		sprintf(speed_info,"flat[%d],speed[%.02f],high[%.02f],time[%.02f]\r\n",g_monitor_info.now_floor,g_monitor_info.flatspeed,flatHigh,speed_end_time);
		rt_kprintf("\r\n%s\r\n",speed_info);
	} 
	
	if ((lastFloorStatus == FLAT_BED) && 
	   (g_monitor_info.floor_status != FLAT_BED))
	{
		lastFloorStatus = g_monitor_info.floor_status;
		speed_start_flag = 1;
		speed_start_time = 0;
	} else if ((lastFloorStatus == NON_FLAT_BED) && 
	   (g_monitor_info.floor_status == FLAT_BED)) 
	{
		speed_start_flag = 0;
		speed_start_time = 0;
		g_monitor_info.flatspeed = 0;
	}


	/*发送故障数据到服务器*/
	/**********冲顶***********/	

	if (g_elevator_flash_info.flat_down == 1) {
		highFlat = g_elevator_flash_info.total_flat;
	} else if (g_elevator_flash_info.flat_down < 0){
		highFlat = g_elevator_flash_info.total_flat + g_elevator_flash_info.flat_down;
	}
	if ((highFlat == g_monitor_info.now_floor) && 
		(g_monitor_info.direction == UP_WALK))
	{
		if (up_walk_time++ > 10*8) {
			up_walk_time = 0;
			set_fault_info(RACE_TOP);
		}
	}
	
	/***********蹲底***********/	
	if ((g_elevator_flash_info.flat_down == g_monitor_info.now_floor) && 
	   (g_monitor_info.direction == DOWN_WALK))
	{
		if (down_walk_time++ > 10*8) {
			down_walk_time = 0;
			set_fault_info(RACE_DOWN);
		}
	}
	
	/***********电梯速度异常***********/
	/***********超速***********/	
	float flatSpeed = 0;
	float fastSpeed = 0;
	
	flatSpeed = (float)g_elevator_flash_info.flat_speed / 100;
	fastSpeed = flatSpeed + flatSpeed*0.15;

	//sprintf(speed_info,"flat[%d],flatSpeed[%.02f],fastSpeed[%.02f],currentSpeed[%.02f]\r\n",g_monitor_info.now_floor,flatSpeed,fastSpeed,g_monitor_info.flatspeed);
	//rt_kprintf("\r\n%s\r\n",speed_info);
	
	if (g_monitor_info.flatspeed > fastSpeed) {
		set_fault_info(SPEED_PASS);
		set_fault_info(ABNORMAL_SPEED);
	}

	/***********门区外停梯***********/		
	if ((g_monitor_info.floor_status == NON_FLAT_BED) && 
		(lastFloorBed == g_monitor_info.now_floor))
	{
		outSideCnt++;
		
		/*如果连续3分钟以上2个条件都满足，则门区外停梯*/
		if (outSideCnt++ > OUT_SIDE_DOOR_TIME) {
			set_fault_info(OUTSIDE_GATE);
		} 
	} else {
		outSideCnt = 0;
	}
	
	/***********停电***********/	
	/*待续*/
	/***********安全回路断路***********/	
	/*暂不实现*/
	/***********开门故障***********/	

	
	if ((g_monitor_info.floor_status == FLAT_BED) &&
	   (lastFloorBed == g_monitor_info.now_floor) && 
	   (lastDoorStatus != g_monitor_info.door_status))
	{	
		door_open_time++;
		door_start_time = 1;

	} else if (g_monitor_info.floor_status != FLAT_BED)
	{
		door_open_time = 0;
		door_start_time = 0;
	}

	if (door_start_time == 1) {
		door_fault_time++;
		if (door_fault_time < 10*60*2) {
			if (door_open_time > 11) {
				door_open_time = 0;
				door_start_time = 0;
				door_fault_time = 0;
				set_fault_info(OPEN_FAULT);
			}
		} else {
			door_open_time = 0;
			door_start_time = 0;
			door_fault_time = 0;
		}
	}
	/***********关门故障：门开超过3分钟不关***********/		
	if ((g_monitor_info.door_status == OPEN) && 
		(lastFloorBed == g_monitor_info.now_floor) && 
	    (g_monitor_info.floor_status == FLAT_BED)){
		door_close_time ++;
		if (door_close_time > CLOSE_DOOR_FAULT_TIME) {
			door_close_time = 0;
			set_fault_info(CLOSE_FAULT);
		} 
	} 

	/***********开门走车***********/	
	uint32_t static open_door_state = 0;
	uint32_t static open_door_time = 0;
	
	if ((g_monitor_info.floor_status == NON_FLAT_BED) && 
		(g_monitor_info.door_status == OPEN) && 
		(lastDoorStatus == OPEN)) 
	{
		open_door_state = 1;
	} else {
		open_door_state = 0;
		open_door_time = 0;
	}

	if (open_door_state == 1 ) {
		if (open_door_time++ > 6) {
			set_fault_info(OPNE_RUN_CAR);			
			open_door_state = 0;
			open_door_time = 0;
		}
	}

	
	/************消除报警************/
	int8_t static last_flat_flag = 0;
	int8_t static lastFloor = 0;
	int8_t static disappear_floor = 0;

	
	if (disappear_floor != g_monitor_info.now_floor) {
	
		if ((lastDoorStatus == CLOSE) &&
		   (g_monitor_info.door_status == OPEN) &&
		   (g_monitor_info.floor_status == FLAT_BED))
		{
			lastFloor = g_monitor_info.now_floor;
			last_flat_flag = 1;
		} else if (g_monitor_info.floor_status != FLAT_BED){
			last_flat_flag = 0;
		}

		if (last_flat_flag == 1) {
			if ((lastDoorStatus == OPEN) && 
			   (g_monitor_info.door_status == CLOSE) &&
			   (g_monitor_info.floor_status == FLAT_BED))
			{
				if (lastFloor == g_monitor_info.now_floor) {
					last_flat_flag = 0;
					disappear_floor = g_monitor_info.now_floor;
					for (i = 1;i < 16;i ++) {
						if (g_fault_list[i].faultID != 0) {
							disappear_info_to_server(g_fault_list[i].faultID);
							g_fault_list[i].faultID = 0;
						}
					}
				}
			} else if (g_monitor_info.floor_status != FLAT_BED) {
				last_flat_flag = 0;
			}
		}
	}
	/***********门锁回路断路***********/	
	/*暂不实现*/
	/***********层站按钮粘连***********/	
	/*暂不实现*/


	/*5秒上报一次monitor数据*/
	if (monitor_time++ > MONITOR_TIME) {
		monitor_time = 0;
		monitor_info_to_server();
	}


#if 1  //chaokw	
	/*10秒上报一次fault数据*/
	if (fault_time++ > FAULT_TIME) {
		fault_time = 0;
		for (i = 1;i < 16;i ++) {
			if (g_fault_list[i].faultID != 0) {
				fault_info_to_server(g_fault_list[i].faultID);
			}
		}
	}
#endif	

	lastFloorStatus = g_monitor_info.floor_status;
	lastFloorBed = g_monitor_info.now_floor;
	lastDoorStatus = g_monitor_info.door_status;
	rt_thread_sleep(10); //100 ms
}


void get_elevator_flash_info(void)
{
	STMFLASH_Read(FLASH_TOTAL_FLAT_ADDR,(uint32_t *)&g_elevator_flash_info.total_flat,1);	
	STMFLASH_Read(FLASH_FLAT_DOWN_ADDR,(uint32_t *)&g_elevator_flash_info.flat_down,1);	
	STMFLASH_Read(FLASH_FLET_BED_HIGH_ADDR,(uint32_t *)&g_elevator_flash_info.flat_bed_high,1);	
	STMFLASH_Read(FLASH_FLET_SPEED_ADDR,(uint32_t *)&g_elevator_flash_info.flat_speed,1);	

#if 0
	if(g_elevator_flash_info.total_flat = -1)     //chaokw
		g_elevator_flash_info.total_flat = 10;
	if(g_elevator_flash_info.flat_bed_high = -1) 
		g_elevator_flash_info.flat_bed_high = 300;
	if(g_elevator_flash_info.flat_speed = -1)
		g_elevator_flash_info.flat_speed = 280;		
#endif	
	//uint8_t total_flat = (uint8_t)g_elevator_flash_info.total_flat & 0xff;
	//flat_bed_cmd_send(SET_FLAT_BED_TOTAL_FLAT_CMD,(uint8_t *)&total_flat,2); //chaokw

	rt_kprintf("\r\nflash info :\r\n[total_flat] -> [%d]\r\n",g_elevator_flash_info.total_flat);
	rt_kprintf("[flat_down] -> [%d]\r\n",g_elevator_flash_info.flat_down);
	rt_kprintf("[flat_bed_high] -> [%d cm]\r\n",g_elevator_flash_info.flat_bed_high);
	rt_kprintf("[flat_speed] -> [%d cm/s]\r\n",g_elevator_flash_info.flat_speed);

	
}


void ele_state_mq(ele_state_t state)
{	
	rt_mq_send(&g_fault_mq, (char *)&state, 1);
}

void fault_report_mq_init(void)
{
	/* 如果有多个线程等待，按照先来先得到的方法分配消息 */

    rt_mq_init(&g_fault_mq, "fault_mq", 
        &g_fault_msg_pool[0], 			
        128 - sizeof(void*), 			
        sizeof(g_fault_msg_pool), 		
        RT_IPC_FLAG_FIFO); 				
}

void fault_handle_fun(char *ele_dat)
{
	uint8_t i;
	static int8_t last_flat_flag = 0;
	static int8_t lastFloor = 0;
	door_status_t static lastdoorStatus = 0;
	
	memcpy((char *)&g_ele_msg,ele_dat,sizeof(e_handle_msg_t));
	

	switch (g_ele_msg.cmd) {
	case DOOR_STATE:
		g_monitor_info.door_status = DOOR_SENSOR_STATE(); 
		rt_kprintf("exit door_status = %d\r\n",g_monitor_info.door_status);
		//get_flat_bed_info();
		
		/***********运行中开门***********/	
		if ((g_monitor_info.floor_status == NON_FLAT_BED) && 
			(g_monitor_info.door_status == OPEN) &&
			(lastdoorStatus == CLOSE))
		{
			set_fault_info(RUNNING_OPEN);
		}

#if 1
		/************消除报警************/
		if ((lastdoorStatus == CLOSE) &&
		   (g_monitor_info.door_status == OPEN) &&
		   (g_monitor_info.floor_status == FLAT_BED))
		{
			lastFloor = g_monitor_info.now_floor;
			last_flat_flag = 1;
		}

		if (last_flat_flag == 1) {
			if ((lastdoorStatus == OPEN) && 
			   (g_monitor_info.door_status == CLOSE) &&
			   (g_monitor_info.floor_status == FLAT_BED))
			{
				if (lastFloor == g_monitor_info.now_floor) {
					last_flat_flag = 0;
					for (i = 1;i < 16;i ++) {
						if (g_fault_list[i].faultID != 0) {
							disappear_info_to_server(g_fault_list[i].faultID);
							g_fault_list[i].faultID = 0;
						}
					}
				}
			}
		}
#endif
		break;


	default:
		break;

	}
	lastdoorStatus = g_monitor_info.door_status;
}

void fault_report_thread_entry(void* parameter)
{
	int result = 0;
    char fault_buf[256] = {0};

	fault_report_mq_init();
	while (1) {
		rt_memset(&fault_buf[0], 0, sizeof(fault_buf));

		result = rt_mq_recv(&g_fault_mq, &fault_buf[0], sizeof(fault_buf), RT_WAITING_FOREVER);
		if (result == RT_EOK) {
			rt_kprintf("fault_buf recv msg: %02x\n", fault_buf[0]);
			fault_handle_fun(fault_buf);
			
		} else if (result == -RT_ETIMEOUT) {
			//rt_kprintf("fault_buf recv msg timeout\n");
		}
	}
}

void monitor_report_thread_entry(void* parameter)
{
	get_elevator_flash_info();
	while (1) {
		monitor_report_handle();
	}
}

void rt_thread_applied_logic_task_init(void)
{
	rt_thread_t device_thread;
	device_thread = rt_thread_create("monitor_thread",
							   monitor_report_thread_entry, RT_NULL,
							   2048, 8, 20);
	if (device_thread != RT_NULL)
		rt_thread_startup(device_thread);	

	device_thread = rt_thread_create("fault_thread",
							   fault_report_thread_entry, RT_NULL,
							   2048, 8, 20);
	if (device_thread != RT_NULL)
		rt_thread_startup(device_thread);	
}

void applied_logic_thread_entry(void* parameter)
{
	flat_fault_timer_init();
	rt_thread_applied_logic_task_init();
}


/*****************************************************************************/
/*    SHELL CMD					                                             */
/*****************************************************************************/
void monitor_info(void)
{
	monitor_report_handle();
}
FINSH_FUNCTION_EXPORT(monitor_info, monitor_info());

void fault_info(uint8_t fault)
{
	fault_info_to_server(FLAT_CLOSE_PEOPLE);
}
FINSH_FUNCTION_EXPORT(fault_info, fault_info(1~15));

void ele_state(uint8_t state)
{
	ele_state_mq(state);
}
FINSH_FUNCTION_EXPORT(ele_state, ele_state(1~15));


/*设置总楼层数*/
void set_total_flat(char data)
{
#if 0
	rt_kprintf("\r\n[total flat] = %d\r\n", data);
	if(data < 1) {   //chaokw
		rt_kprintf("\r\nset error: total flat should be positive!\r\n");
		return;
	}
#endif	
	flat_bed_cmd_send(SET_FLAT_BED_TOTAL_FLAT_CMD,(uint8_t *)&data,2);
	STMFLASH_Write(FLASH_TOTAL_FLAT_ADDR,(uint32_t *)&data,1); 	
}
FINSH_FUNCTION_EXPORT(set_total_flat, set_total_flat(10));


/*设置楼层的最低层，如果最低是-2层，填写-2，如果最低是1层，填写1*/
void set_flat_down(char flat)
{
	STMFLASH_Write(FLASH_FLAT_DOWN_ADDR,(uint32_t *)&flat,1); 	
}
FINSH_FUNCTION_EXPORT(set_flat_down, set_flat_down(-1));


/*设置两个楼层间的高度，测速用的,单位是厘米*/
void set_flat_high(uint32_t high)
{
	STMFLASH_Write(FLASH_FLET_BED_HIGH_ADDR,(uint32_t *)&high,1); 	
}
FINSH_FUNCTION_EXPORT(set_flat_high, set_flat_high(300) Unit is cm );


/*设置电梯的额定速度，单位cm/s*/
void set_flat_speed(uint32_t high)
{
	STMFLASH_Write(FLASH_FLET_SPEED_ADDR,(uint32_t *)&high,1); 	
}
FINSH_FUNCTION_EXPORT(set_flat_speed, set_flat_speed(280) Unit is cm/s );


#define ELEVATOR_VERSION                1L              /**< major version number */  //chaokw
#define ELEVATOR_SUBVERSION             1L              /**< minor version number */
long get_version(void)
{
    rt_kprintf("ELEVATOR IF812    V%d.%d build %s\n",
               ELEVATOR_VERSION, ELEVATOR_SUBVERSION, __DATE__);
    return 0;
}
FINSH_FUNCTION_EXPORT(get_version, show ELEVATOR IF812 version information);


void get_config(void)
{
	rt_kprintf("\r\nflash info :\r\n[total_flat] -> [%d]\r\n",g_elevator_flash_info.total_flat);
	rt_kprintf("[flat_down] -> [%d]\r\n",g_elevator_flash_info.flat_down);
	rt_kprintf("[flat_bed_high] -> [%d cm]\r\n",g_elevator_flash_info.flat_bed_high);
	rt_kprintf("[flat_speed] -> [%d cm/s]\r\n",g_elevator_flash_info.flat_speed);
}
FINSH_FUNCTION_EXPORT(get_config, show ELEVATOR IF812 config information);



