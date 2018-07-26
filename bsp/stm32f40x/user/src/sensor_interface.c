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

/*****************************************************************************/
/*    DEFINE DECLARATIONS                                                    */
/*****************************************************************************/

static rt_sem_t g_flat_bed_sensor_sem;
static rt_sem_t g_uart_send_flat_sem;

rt_device_t g_flat_bed_sensor_device;

extern monitor_info_t g_monitor_info;
extern elevator_flash_info_t g_elevator_flash_info;
/*****************************************************************************/
/*    PRIVATE FUNCTION DECLARATIONS                                          */
/*****************************************************************************/
typedef struct {
	uint8_t headH;
	uint8_t headL;
	uint8_t len;
	uint8_t cmd;
	uint8_t data[256];
}flat_bed_t;


void HexToStr(uint8_t *pbDest, uint8_t *pbSrc, int nLen)
{
	char ddl,ddh;
	int i;

	for (i=0; i<nLen; i++) {
		ddh = 48 + pbSrc[i] / 16;
		ddl = 48 + pbSrc[i] % 16;
		if (ddh > 57) ddh = ddh + 7;
		if (ddl > 57) ddl = ddl + 7;
		pbDest[i*2] = ddh;
		pbDest[i*2+1] = ddl;
	}

	pbDest[nLen*2] = '\0';
}

void StrToHex(uint8_t *pbDest, uint8_t *pbSrc, int nLen)
{
	uint8_t h1,h2;
	uint8_t s1,s2;
	int i;

	for (i=0; i<nLen; i++) {
		h1 = pbSrc[2*i];
		h2 = pbSrc[2*i+1];

		s1 = toupper(h1) - 0x30;
		if (s1 > 9) 
			s1 -= 7;

		s2 = toupper(h2) - 0x30;
		if (s2 > 9) 
			s2 -= 7;

		pbDest[i] = s1*16 + s2;
	}
}


unsigned char flat_bed_crc8(volatile unsigned char *ptr, unsigned char len)
{
	unsigned char i;
	unsigned char crc = 0;
	while(len--!=0) {
		for(i=1; i!=0; i*=2) {
			if((crc&1)!=0) {
				crc/=2; crc^=0x8C;
			} else {
				crc/=2;
			}
			
			if((*ptr&i)!=0) 
				crc^=0x8C;
		}
		ptr++;
	}
	return(crc);
}


/*
	PD8 -> radar I/O
*/
static void radar_sensor_exti9_5_gpio_init(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;
	
	/* Enable GPIOD clk */
 	 RCC_AHB1PeriphClockCmd(RADAR_SENSOR_RCC_AHB1, ENABLE);	
	
	/* Configure PD8 pin as input floating */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;  
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_InitStructure.GPIO_Pin = RADAR_SENSOR_EXTI_GPIO_PIN;	  
	GPIO_Init(RADAR_SENSOR_EXTI_GPIO_PORT, &GPIO_InitStructure);
}

static void radar_sensor_exti9_5_mode_config(void)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RADAR_SENSOR_RCC_APB2, ENABLE); 

	/* Connect EXTI Line8 to PD8 pin */
	SYSCFG_EXTILineConfig(RADAR_SENSOR_EXTI_SOURCE_GPIO_PORT,\
						  RADAR_SENSOR_EXTI_SOURCE_GPIO_PIN);

	/* Configure EXTI Line8 */
	EXTI_InitStructure.EXTI_Line = RADAR_SENSOR_EXTI_LINE;			  
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	  
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;				  
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI Line8 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = RADAR_SENSOR_NVIC_IRQ_CHANNEL;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}


/*
	PD12 -> radar I/O
*/
static void door_sensor_exti15_10_gpio_init(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;
	
	/* Enable GPIOD clk */
 	 RCC_AHB1PeriphClockCmd(RADAR_SENSOR_RCC_AHB1, ENABLE);	
	
	/* Configure PD8 pin as input floating */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;  
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_InitStructure.GPIO_Pin = DOOR_SENSOR_EXTI_GPIO_PIN;	  
	GPIO_Init(DOOR_SENSOR_EXTI_GPIO_PORT, &GPIO_InitStructure);
}

static void door_sensor_exti15_10_mode_config(void)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(DOOR_SENSOR_RCC_APB2, ENABLE); 

	/* Connect EXTI Line8 to PD12 pin */
	SYSCFG_EXTILineConfig(DOOR_SENSOR_EXTI_SOURCE_GPIO_PORT,\
						  DOOR_SENSOR_EXTI_SOURCE_GPIO_PIN);

	/* Configure EXTI Line12 */
	EXTI_InitStructure.EXTI_Line = DOOR_SENSOR_EXTI_LINE;			  
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	  
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;				  
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI Line12 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = DOOR_SENSOR_NVIC_IRQ_CHANNEL;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

void radar_sensor_init(void)
{
	radar_sensor_exti9_5_gpio_init();		
	radar_sensor_exti9_5_mode_config();	
}

void door_sensor_init(void)
{
	door_sensor_exti15_10_gpio_init();		
	door_sensor_exti15_10_mode_config();	
}


rt_err_t uart4_flat_bed_sensor_input(rt_device_t dev, rt_size_t size)
{
	rt_err_t ret = 0;
	ret = rt_sem_release(g_flat_bed_sensor_sem);
	return ret;
}

int rt_flat_bed_sensor_sem_init(void)
{
	g_flat_bed_sensor_sem = rt_sem_create
	(
		"uart_recv_flat_bed_sensor_sem", 
		1, 
		RT_IPC_FLAG_FIFO
	);
	if(g_flat_bed_sensor_sem == RT_NULL){
		rt_kprintf("semaphore creation failed\n");
		return -1;
	}

	g_uart_send_flat_sem = rt_sem_create
	(
		"g_uart_send_flat_sem", 
		1, 
		RT_IPC_FLAG_FIFO
	);
	if(g_uart_send_flat_sem == RT_NULL){
		rt_kprintf("semaphore creation failed\n");
		return -1;
	}

	
	return 0;
}

void flat_bed_485_uart_input(void)
{
	flat_bed_gpio_low();
}

void flat_bed_485_uart_output(void)
{
	flat_bed_gpio_high();
}


void flat_bed_485_uart_GPIO_init(void)
{
	GPIO_InitTypeDef GpioInitStructer;
	
	RCC_APB1PeriphClockCmd( FLAT_BED_485_CLK, ENABLE );
		
	GpioInitStructer.GPIO_Pin = FLAT_BED_485_PIN;
	GpioInitStructer.GPIO_Mode = GPIO_Mode_OUT;
	GpioInitStructer.GPIO_Speed = GPIO_Speed_50MHz;
	GpioInitStructer.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GpioInitStructer.GPIO_OType = GPIO_OType_PP;
	
	GPIO_Init( FLAT_BED_485_PORT, &GpioInitStructer );
	
	flat_bed_485_uart_input();

}

void flat_bed_sensor_init(void)
{
	g_flat_bed_sensor_device = rt_device_find("uart4");
	if(g_flat_bed_sensor_device != RT_NULL){
		rt_device_open(g_flat_bed_sensor_device, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
	}
	
	rt_device_set_rx_indicate(g_flat_bed_sensor_device,uart4_flat_bed_sensor_input);
	rt_flat_bed_sensor_sem_init();

	flat_bed_485_uart_GPIO_init();
}

/*
	人体雷达,门磁传感,平层传感初始化
*/
rt_err_t sensor_init(void)
{
	rt_err_t ret = 0;
	
	radar_sensor_init();		//PD8  -> radar I/O
	door_sensor_init();			//PD12 -> door sensor I/O
	flat_bed_sensor_init();		//uart4
	
	return ret;
}


void flat_bed_device_write(const uint8_t *buffer, rt_size_t size)
{
	/*485发送状态*/
	flat_bed_485_uart_output();
	rt_device_write(g_flat_bed_sensor_device, 0, buffer, size);
	
	while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET); 
	while(USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET);	
	
	/*485接收状态*/
	flat_bed_485_uart_input();
}


void hex_dump(uint8_t *buf, int len, char *str)
{
    int i;

	//rt_kprintf("\r\n==============%s==============\n", str);
	rt_kprintf("\r\n%s:\n", str);
    for (i = 0; i < len; i++) {
        rt_kprintf("%02X ", buf[i]);
        if (i != 0 && (0 == i % 16)) {
            rt_kprintf("\n");
        }
    }
	rt_kprintf("\n");
    //rt_kprintf("\n===============DUMP END====================\n\n");
}



void handle_process(uint8_t *data,uint8_t len)
{
	int8_t nowFloor = 0;
	switch(data[3]) {
	case RETURN_FLAT_BED_INFO_CMD:
		nowFloor = data[4] + g_elevator_flash_info.flat_down;

		if (g_elevator_flash_info.flat_down == 1) {		
			g_monitor_info.now_floor = nowFloor;
			
		} else if (g_elevator_flash_info.flat_down < 0) {
			if (nowFloor >= 0 ) {
				g_monitor_info.now_floor = nowFloor + 1;
			} else {
				g_monitor_info.now_floor = nowFloor;
			}
		}
		
		g_monitor_info.direction = data[5];

		//rt_kprintf("flat[%d],state[%d] -> 0:stop,1:up,2:down\r\n",g_monitor_info.now_floor,g_monitor_info.direction);
		break;

	/*case ..
		break;*/

	default:
		break;
	}
}


/*
	55 AA 03 50 10 00 00
*/
int flat_bed_sensor_push_buff(uint8_t uartDat)
{
	int i = 0;
	int ret = 0;
	static uint8_t count = 0;
	static uint8_t uartBuf[300] = {0};
	uint8_t crc = 0;

	//rt_kprintf("[%02x] ",uartDat);
	uartBuf[count] = uartDat;
	if (uartBuf[0] == FLAT_BED_HEAD_H) {
		count++;
		if (count < 3) {
			return 0;
		}
		if (uartBuf[2] >= (count-3)) {
			return 0;
		}

		crc = flat_bed_crc8((char *)&uartBuf[2], uartBuf[2]+1);
		if (crc != uartBuf[count-1]) {
			rt_kprintf("flat checksum is error!\r\n");
			count = 0;
			rt_memset(uartBuf, 0, sizeof(uartBuf));
		}

		//hex_dump(uartBuf, count, "flat bed (recv...)");

		handle_process(uartBuf,count);
		
		count = 0;
		rt_memset(uartBuf, 0, sizeof(uartBuf));	
		
	} else if (uartBuf[0] == FLAT_BED_HEAD_ACK){
		rt_kprintf("\r\nset total data to flat sensor\r\n");
	} else {
		count = 0;
		rt_memset(uartBuf, 0, sizeof(uartBuf));

	}
	
		
	return 0;
}

/*
	SET_FLAT_BED_TOTAL_FLAT_CMD
	GET_FLET_BED_INFO_CMD

	55 AA 02 40 0F 95 
	55 AA 01 46    5F
*/
int flat_bed_cmd_send(uint8_t cmd,uint8_t *data,uint8_t len)
{
	uint8_t crc = 0;
	flat_bed_t flat_bed_cmd;
	flat_bed_cmd.headH = FLAT_BED_HEAD_H;
	flat_bed_cmd.headL = FLAT_BED_HEAD_L;
	flat_bed_cmd.len = len;
	flat_bed_cmd.cmd = cmd;
	
	memcpy(flat_bed_cmd.data,data,len);
	crc = flat_bed_crc8((uint8_t *)&flat_bed_cmd.len,len+1);
		
	*((uint8_t *)flat_bed_cmd.data + len-1) = crc;
	//hex_dump((uint8_t *)&flat_bed_cmd,4 + len,"flat bed (send...)");

	flat_bed_device_write((uint8_t *)&flat_bed_cmd,4 + len);
}

int get_flat_bed_info(void)
{
	//rt_sem_take(g_uart_send_flat_sem,100 * 5);
	flat_bed_cmd_send(GET_FLET_BED_INFO_CMD,NULL,1);
	//rt_sem_release(g_uart_send_flat_sem);
}


void sensor_thread_entry(void* parameter)
{
	rt_err_t ret;
	uint8_t uartDat = 0;
	uint8_t rxLen = 0;

	ret = sensor_init();
	if (ret != RT_EOK) {
		rt_kprintf("\r\nsensor_init failed!\r\n");
	}

	while (1) {
		ret = rt_sem_take(g_flat_bed_sensor_sem,RT_WAITING_FOREVER);
		if (ret != RT_EOK) {
			 rt_kprintf("Failed to take semaphore.\n");
		}
		do {
			rxLen = rt_device_read(g_flat_bed_sensor_device, 0, &uartDat, 1);
			if (rxLen == 0) {
				break;
			}
			
			ret = flat_bed_sensor_push_buff(uartDat); 
			if (ret < 0) {
				rt_kprintf("flat bed sensor data error.\n");
			}
		} while (1);	
	}
}


/*****************************************************************************/
/*    SHELL CMD					                                             */
/*****************************************************************************/
void flat_bed_data(char *dat)
{
	uint8_t i;
	uint8_t len = 0;
	uint8_t lora_data[254] = {0};

	memset(lora_data,0,sizeof(lora_data));
	
	len = strlen(dat)/2;
	
	rt_kprintf("data = %s, len = %d\r\n",dat,len);
	for (i = 0;i < len;i ++) {
		StrToHex(&lora_data[i], dat + i*2, 1);
		rt_kprintf("%02x\r\n",lora_data[i]);
	}
	flat_bed_device_write(lora_data,len);
}
FINSH_FUNCTION_EXPORT(flat_bed_data, flat_bed_data("55aa01465f"));


/*获取平层传感器的数据*/
void get_flat_info(void)
{
	get_flat_bed_info();
}
FINSH_FUNCTION_EXPORT(get_flat_info, get_flat_info());

