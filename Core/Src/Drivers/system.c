/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * system.c - Top level module implementation
 */
#define DEBUG_MODULE "SYS"

#include <stdbool.h>

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

//#include "debug.h"
//#include "version.h"
#include "config.h"
#include "led.h"
//#include "param.h"
//#include "log.h"
//#include "ledseq.h"
//#include "pm.h"

#include "system.h"
#include "platform.h"
//#include "storage.h"
//#include "configblock.h"
//#include "worker.h"
//#include "freeRTOSdebug.h"
//#include "uart_syslink.h"
//#include "uart1.h"
//#include "uart2.h"
//#include "comm.h"
//#include "stabilizer.h"
//#include "commander.h"
//#include "console.h"
//#include "usblink.h"
//#include "mem.h"
//#include "crtp_mem.h"
//#include "proximity.h"
//#include "watchdog.h"
//#include "queuemonitor.h"
//#include "buzzer.h"
//#include "sound.h"
//#include "sysload.h"
//#include "estimator_kalman.h"
//#include "estimator_ukf.h"
//#include "deck.h"
//#include "extrx.h"
//#include "app.h"
#include "static_mem.h"
//#include "peer_localization.h"
#include "cfassert.h"
//#include "i2cdev.h"
#include "autoconf.h"
//#include "vcp_esc_passthrough.h"
#if CONFIG_ENABLE_CPX
//  #include "cpxlink.h"
#endif

/* Private variable */
static bool selftestPassed;
static uint8_t dumpAssertInfo = 0;
static bool isInit;

static char nrf_version[16];
static uint8_t testLogParam;
static uint8_t doAssert;

STATIC_MEM_TASK_ALLOC(systemTask, SYSTEM_TASK_STACKSIZE);

/* System wide synchronisation */
xSemaphoreHandle canStartMutex;
static StaticSemaphore_t canStartMutexBuffer;

/* Private functions */
static void systemTask(void *arg);

/* Public functions */
void systemLaunch(void)
{
  STATIC_MEM_TASK_CREATE(systemTask, systemTask, SYSTEM_TASK_NAME, NULL, SYSTEM_TASK_PRI);
}

/* Private functions implementation */

void systemTask(void *arg)
{
  bool pass = true;

  ledInit();
  ledSet(CHG_LED, 1);

#ifdef CONFIG_DEBUG_QUEUE_MONITOR
//  queueMonitorInit();
#endif

#ifdef CONFIG_DEBUG_PRINT_ON_UART1 // UART6 사용
  MX_USART6_UART_Init();
//  uart1Init(CONFIG_DEBUG_PRINT_ON_UART1_BAUDRATE);
#endif



}




