
#include <stdio.h>
#include <string.h>
#include "main.h"
//#include "stm32f4xx_hal_uart.h"
//#include "stm32f4xx_hal_def.h"

extern UART_HandleTypeDef huart6;

int _write(int file, char* p, int len)
{
		if(HAL_UART_Transmit(&huart6, p, len , 10 )!= 0)
			//		while(!LL_USART_IsActiveFlag_TXE(USART6));0
			//		usDelay(100);	// 문자 1개 출력당 약 100us 소요, Float, int형 차이 없음
			return -1;
		return len;
}
