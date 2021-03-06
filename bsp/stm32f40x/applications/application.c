/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2014-04-27     Bernard      make code cleanup. 
 */

#include <board.h>
#include <rtthread.h>

#ifdef RT_USING_LWIP
#include <lwip/sys.h>
#include <lwip/api.h>
#include <netif/ethernetif.h>
#include "stm32f4xx_eth.h"
#endif

#ifdef RT_USING_FINSH
#include <shell.h>
#include <finsh.h>
#endif

#ifdef RT_USING_GDB
#include <gdb_stub.h>
#endif

#include "sensor_interface.h"
#include "monitor_interface.h"
#include "applied_logic_interface.h"

void rt_init_thread_entry(void* parameter)
{
    /* GDB STUB */
#ifdef RT_USING_GDB
    gdb_set_device("uart6");
    gdb_start();
#endif

    /* LwIP Initialization */
#ifdef RT_USING_LWIP
    {
        extern void lwip_sys_init(void);

        /* register ethernetif device */
        eth_system_device_init();

        rt_hw_stm32_eth_init();

        /* init lwip system */
        lwip_sys_init();
        rt_kprintf("TCP/IP initialized!\n");
    }
#endif

#ifdef RT_USING_FINSH
	/* init finsh */
	finsh_system_init();
#endif
}



void elevator_sys_init(void)
{
	rt_thread_t device_thread;

	device_thread = rt_thread_create("sensor_device",
							   sensor_thread_entry, RT_NULL,
							   2048, 8, 20);
	if (device_thread != RT_NULL)
		rt_thread_startup(device_thread);


	device_thread = rt_thread_create("monitor_device",
							   monitor_thread_entry, RT_NULL,
							   2048, 8, 20);
	if (device_thread != RT_NULL)
		rt_thread_startup(device_thread);

	device_thread = rt_thread_create("applied_logic",
							   applied_logic_thread_entry, RT_NULL,
							   2048, 8, 20);
	if (device_thread != RT_NULL)
		rt_thread_startup(device_thread);	

}

int rt_application_init()
{
    rt_thread_t tid;

	elevator_sys_init();

    tid = rt_thread_create("init",
        rt_init_thread_entry, RT_NULL,
        2048, RT_THREAD_PRIORITY_MAX/3, 20);

    if (tid != RT_NULL)
        rt_thread_startup(tid);

    return 0;
}
