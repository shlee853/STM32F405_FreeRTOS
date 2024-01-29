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

#include "debug.h"
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
#include "queuemonitor.h"
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


#include "ICM20602.h"


/* Private variable */
static bool selftestPassed;
static uint8_t dumpAssertInfo = 0;
static bool isInit;

static char nrf_version[16];
static uint8_t testLogParam;
static uint8_t doAssert;


unsigned long  time1=0;
unsigned long  time2=0;

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

  uint32_t ld = SysTick->LOAD;
  time1 = DWT->CYCCNT;
  delay_us(10);	// 1ms
  time2 = DWT->CYCCNT;
  printf("delay = %.2f(us)\n",(float)(time2-time1)/CLOCK_PER_USEC);


  ledInit();
  ledSet(CHG_LED, SET);


#ifdef CONFIG_DEBUG_QUEUE_MONITOR
  queueMonitorInit();
#endif

  ICM20602_Initialization();

  passthroughInit();	// Create passthrough task

  systemInit();
//  commInit();
//  commanderInit();

}



void systemWaitStart(void)
{
  //This permits to guarantee that the system task is initialized before other
  //tasks waits for the start event.
  while(!isInit)
    vTaskDelay(2);

  xSemaphoreTake(canStartMutex, portMAX_DELAY);
  xSemaphoreGive(canStartMutex);
}



// This must be the first module to be initialized!
void systemInit(void)
{
  if(isInit)
    return;

  canStartMutex = xSemaphoreCreateMutexStatic(&canStartMutexBuffer);
  xSemaphoreTake(canStartMutex, portMAX_DELAY);

//  usblinkInit();
//  sysLoadInit();
#if CONFIG_ENABLE_CPX
//  cpxlinkInit();
#endif

  /* Initialized here so that DEBUG_PRINT (buffered) can be used early */
//  debugInit();
//  crtpInit();
//  consoleInit();

  DEBUG_PRINT("----------------------------\n");
//  DEBUG_PRINT("%s is up and running!\n", platformConfigGetDeviceTypeName());

/*  if (V_PRODUCTION_RELEASE) {
    DEBUG_PRINT("Production release %s\n", V_STAG);
  } else {
    DEBUG_PRINT("Build %s:%s (%s) %s\n", V_SLOCAL_REVISION,
                V_SREVISION, V_STAG, (V_MODIFIED)?"MODIFIED":"CLEAN");
  }
  DEBUG_PRINT("I am 0x%08X%08X%08X and I have %dKB of flash!\n",
              *((int*)(MCU_ID_ADDRESS+8)), *((int*)(MCU_ID_ADDRESS+4)),
              *((int*)(MCU_ID_ADDRESS+0)), *((short*)(MCU_FLASH_SIZE_ADDRESS)));

  configblockInit();
  storageInit();
  workerInit();
  adcInit();
  ledseqInit();
  pmInit();
  buzzerInit();
  peerLocalizationInit();

#ifdef CONFIG_APP_ENABLE
  appInit();
#endif
*/
  isInit = true;
}



