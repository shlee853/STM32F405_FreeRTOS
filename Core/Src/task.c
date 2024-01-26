/*
 * task.c
 *
 *  Created on: Dec 22, 2020
 *      Author: YUNGKI HONG (guileschool@gmail.com)
 *      Copyright © 2015 guileschool
 */

/* FreeRTOS.org includes. */
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>

//#define CMSIS_OS

/* task's priority */
#define TASK_MAIN_PRIO	20
#define TASK_1_PRIO		10
#define TASK_2_PRIO		 9
#define TASK_3_PRIO		 8

struct Param_types {	/* struct for parameter passing to task */
       char *msg;
       int  P1,P2;
} Param_Tbl;

/* The task functions. */
static void TaskMain( void const *pvParameters );
static void Task1( void const *pvParameters );
static void Task2( const struct Param_types *Param );

#ifdef CMSIS_OS
osThreadId defaultTaskHandle;
osThreadId xHandleMain, xHandle1, xHandle2;
#else
TaskHandle_t xHandleMain, xHandle1, xHandle2;
#endif

int	task1timer, task2timer;

/*-----------------------------------------------------------*/

void USER_THREADS( void )
{

	/* Setup the hardware for use with the Beagleboard. */
	//prvSetupHardware();
#ifdef CMSIS_OS
	osThreadDef(defaultTask, TaskMain, osPriorityHigh, 0, 256);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
#else
	/* Create one of the two tasks. */
	xTaskCreate(	(TaskFunction_t)TaskMain,		/* Pointer to the function that implements the task. */
					"TaskMain",	/* Text name for the task.  This is to facilitate debugging only. */
					256,		/* Stack depth - most small microcontrollers will use much less stack than this. */
					NULL,		/* We are not using the task parameter. */
					TASK_MAIN_PRIO,	/* This task will run at this priority */
					&xHandleMain );		/* We are not using the task handle. */
#endif
}

static void TaskMain( void const *pvParameters )
{
	const char *pcTaskName = "TaskMain";
	struct Param_types *Param;

	pvParameters = pvParameters; // for compiler warning

	/* Print out the name of this task. */
	printf( "%s is running\r\n", pcTaskName );

	// TASK CREATE
	/* TODO #1:
		Task1을 생성
		use 'xTaskCreate' */
#if 0

#endif // TODO #1

	/* Create the other task in exactly the same way. */
	Param = &Param_Tbl;		/* get parameter tbl addr */
	Param->P1 = 111111;		/* set parameter */
	Param->P2 = 222222;
#ifdef CMSIS_OS
	osThreadDef(Task2, (void const *)Task2, osPriorityBelowNormal, 0, 256);
	xHandle2 = osThreadCreate (osThread(Task2), (void*)Param);
#else
	xTaskCreate( (TaskFunction_t)Task2, "Task2", 256, (void*)Param, TASK_2_PRIO, &xHandle2 );
#endif

	/* TODO #2:
		Task1을 중지
		use 'vTaskSuspend' */
#if 0

#endif // TODO #2

	/* TODO #4:
		Task1의 우선 순위를 'TASK_3_PRIO' 으로 변경
		use 'vTaskPrioritySet' and 'vTaskResume' */
#if 0

#endif // TODO #4

	/* delete self task */
	vTaskDelete (xHandleMain);	// vTaskDelete (NULL);
}

static void Task1( void const *pvParameters )
{
	const char *pcTaskName = "Task1";

	pvParameters = pvParameters; // for compiler warning

	/* Print out the name of this task. */
	printf( "%s is running\n", pcTaskName );

	printf("\n-------  Task1 information -------\n");
	printf("task1 name = %s \n",pcTaskGetName( xHandle1 ));
	printf("task1 priority = %d \n",(int)uxTaskPriorityGet( xHandle1 ));
//	printf("task1 status = %d \n",eTaskGetState( xHandle1 ));
	printf("----------------------------------\n");

	while(1) {
	/* TODO #3:
		코드를 실행 하여 보고
		vTaskDelay() 코드를 주석 처리한 후 그 결과를 설명한다 */
#if 0 // No comment
vTaskDelay (pdMS_TO_TICKS (1000));
printf("a"); fflush(stdout);	// 문자 'a' 출력
#endif // TODO #3

		task1timer++;
	}
}

static void Task2( const struct Param_types *Param )
{
	const char *pcTaskName = "Task2";

	/* Print out the name of this task. */
	printf( "%s is running\n", pcTaskName );

	printf("\n-------  Task2 parameter passed from main --------\n");
	printf("task2 first parameter = %d \n",Param->P1);
	printf("task2 second parameter = %d \n",Param->P2);
	printf("--------------------------------------------------\n");

	while(1) {
	/* TODO #3:
		코드를 실행 하여 보고
		vTaskDelay() 코드를 주석 처리한 후 그 결과를 설명한다 */
#if 0 // No comment
vTaskDelay (pdMS_TO_TICKS (1000));
printf("b"); fflush(stdout);	// 문자 'a' 출력
#endif // TODO #3

		task2timer++;
	}
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook (void)
{
	printf("."); fflush(stdout);
}


