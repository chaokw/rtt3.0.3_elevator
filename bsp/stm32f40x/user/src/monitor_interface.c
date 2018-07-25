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
#include "monitor_interface.h"
#include "sensor_interface.h"
#include "applied_logic_interface.h"
/*****************************************************************************/
/*    DEFINE DECLARATIONS                                                    */
/*****************************************************************************/
monitor_info_t g_monitor_info;
static rt_sem_t g_uart_send_monitor_sem;

extern fault_list_t g_fault_list[16];
extern elevator_flash_info_t g_elevator_flash_info;  //chaokw
/*****************************************************************************/
/*    PRIVATE FUNCTION DECLARATIONS                                          */
/*****************************************************************************/
static rt_sem_t g_monitor_sem;
rt_device_t g_monitor_device;

rt_err_t uart2_monitor_input(rt_device_t dev, rt_size_t size)
{
	rt_err_t ret = 0;
	ret = rt_sem_release(g_monitor_sem);
	return ret;
}


int rt_monitor_sem_init(void)
{
	g_monitor_sem = rt_sem_create
	(
		"uart_recv_monitor_sem", 
		1, 
		RT_IPC_FLAG_FIFO
	);
	if(g_monitor_sem == RT_NULL){
		rt_kprintf("semaphore creation failed\n");
		return -1;
	}

	g_uart_send_monitor_sem = rt_sem_create
	(
		"g_uart_send_monitor_sem", 
		1, 
		RT_IPC_FLAG_FIFO
	);
	if(g_uart_send_monitor_sem == RT_NULL){
		rt_kprintf("semaphore creation failed\n");
		return -1;
	}

	return 0;
}

void monitor_485_uart_input(void)
{
	monitor_gpio_low();
}

void monitor_485_uart_output(void)
{
	monitor_gpio_high();
}

void monior_485_uart_GPIO_init(void) 
{
	GPIO_InitTypeDef GpioInitStructer;
	
	RCC_APB1PeriphClockCmd( MONITOR_485_CLK, ENABLE );
		
	GpioInitStructer.GPIO_Pin = MONITOR_485_PIN;
	GpioInitStructer.GPIO_Mode = GPIO_Mode_OUT;
	GpioInitStructer.GPIO_Speed = GPIO_Speed_50MHz;
	GpioInitStructer.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GpioInitStructer.GPIO_OType = GPIO_OType_PP;
	
	GPIO_Init( MONITOR_485_PORT, &GpioInitStructer );
	
	monitor_485_uart_input();
}

void monitor_info_init(void)
{
	g_monitor_info.now_status = 0;
	g_monitor_info.direction = 0;
	g_monitor_info.floor_status = 0;
	g_monitor_info.flatspeed = 0;
	g_monitor_info.door_status = 0;
	g_monitor_info.has_people = 0;
	g_monitor_info.power_status = 0;
	//g_monitor_info.now_floor = 0;
	g_monitor_info.now_floor = g_elevator_flash_info.flat_down;  //chaokw
	g_monitor_info.version = 1;
}

rt_err_t monitor_init(void)
{
	rt_err_t ret = 0;

	g_monitor_device = rt_device_find("uart2");
	if(g_monitor_device != RT_NULL){
		rt_device_open(g_monitor_device, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
	}
	
	rt_device_set_rx_indicate(g_monitor_device,uart2_monitor_input);
	ret = rt_monitor_sem_init();

	monior_485_uart_GPIO_init();

	monitor_info_init();
	return ret;
}


void monitor_device_write(const uint8_t *buffer, rt_size_t size)
{
	rt_sem_take(g_uart_send_monitor_sem,100 * 5);

	/*485发送状态*/
	monitor_485_uart_output();
	rt_device_write(g_monitor_device, 0, buffer, size);
	
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); 
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);	
	
	/*485接收状态*/
	monitor_485_uart_input();
	
	rt_sem_release(g_uart_send_monitor_sem);
}

/*
#10#{
	"type": "Monitor",
	"stype": "status",
	"now_status": "1",
	"direction": "2",
	"floor_status": "3",
	"speed": "1.12",
	"door_status": "5",
	"has_people": "6",
	"power_status": "7",
	"now_floor": "8",
	"version": "1"
}#13#
*/

void monitor_info_to_server(void)
{
	
	char monitor_info[200] = {0};

	if (g_monitor_info.now_floor == 0)   //chaokw
		g_monitor_info.now_floor = g_elevator_flash_info.flat_down;
	sprintf(monitor_info,"#10#{\"type\":\"Monitor\",\"stype\":\"status\",\"now_status\":%d,\"direction\":%d,\"floor_status\":%d,\"speed\":%.02f,\"door_status\":%d,\"has_people\":%d,\"power_status\":%d,\"now_floor\":%d,\"version\":%d}#13#",
		g_monitor_info.now_status,
		g_monitor_info.direction,
		g_monitor_info.floor_status,
		g_monitor_info.flatspeed,
		g_monitor_info.door_status,
		g_monitor_info.has_people,
		g_monitor_info.power_status,
		g_monitor_info.now_floor,
		g_monitor_info.version);

	monitor_device_write(monitor_info,strlen(monitor_info));
	rt_kprintf("\r\n%s",monitor_info);
}
 
/*
{
	"type": "Fault",
	"stype": "add",
	"fault_type": 1,
	"floor": 0,
	"direction": 0,
	"speed": 536888232
}	
*/
void fault_info_to_server(fault_info_t faultID)
{
	char fault_info[200] = {0};

	if (g_monitor_info.now_floor == 0)   //chaokw
		g_monitor_info.now_floor = g_elevator_flash_info.flat_down;
	sprintf(fault_info,"#10#{\"type\":\"Fault\",\"stype\":\"add\",\"fault_type\":%d,\"floor\":%d,\"direction\":%d,\"speed\":%.02f}#13#",
		g_fault_list[faultID].faultID,g_fault_list[faultID].faultFloor,g_fault_list[faultID].faultDirection,g_fault_list[faultID].faultSpeed);

	monitor_device_write(fault_info,strlen(fault_info));
	rt_kprintf("\r\n%s",fault_info);
}


/*
	{"type":"Fault","stype":"disappear","fault_type":1}
*/

void disappear_info_to_server(fault_info_t faultID)
{
	char disappear_info[200] = {0};
	sprintf(disappear_info,"#10#{\"type\":\"Fault\",\"stype\":\"disappear\",\"fault_type\":%d}#13#",faultID);
	
	monitor_device_write(disappear_info,strlen(disappear_info));
	
	rt_kprintf("\r\n%s",disappear_info);
}

int monitor_push_buff(uint8_t uartDat)
{
		
	rt_kprintf("\r\n%02x",uartDat);
	
	/*
		处理monitor下行数据
		...
	*/
}


void monitor_thread_entry(void* parameter)
{
	uint8_t Dat[10] = {0};
	rt_err_t ret;
	uint8_t uartDat = 0;
	uint8_t rxLen = 0;
	ret = monitor_init();
	if (ret != RT_EOK) {
		rt_kprintf("\r\nmonitor_init failed!\r\n");
	}

	while (1) {
		ret = rt_sem_take(g_monitor_sem,RT_WAITING_FOREVER);
		if (ret != RT_EOK) {
			 rt_kprintf("Failed to take semaphore.\n");
		}
		do {
			rxLen = rt_device_read(g_monitor_device, 0, &uartDat, 1);
			if (rxLen == 0) {
				break;
			}
			
			ret = monitor_push_buff(uartDat); 
			if (ret < 0) {
				rt_kprintf("monitor data error.\n");
			}
		} while (1);	
	}
}


/*****************************************************************************/
/*    SHELL CMD					                                             */
/*****************************************************************************/

void monitor_send(char *dat)
{
	monitor_info_to_server();
}
FINSH_FUNCTION_EXPORT(monitor_send, send monitor data to server);

void monitor_test(char *dat)
{
	uint8_t *data = "{\"type\":\"Monitor\",\"stype\":\"status\",\"now_status\":1,\"direction\":1,\"floor_status\":1,\"speed\":1.11,\"door_status\":1,\"has_people\":1,\"power_status\":1,\"now_floor\":1,\"version\":1}";

	monitor_device_write(data,strlen(data));
}
FINSH_FUNCTION_EXPORT(monitor_test, send test monitor data to server);





