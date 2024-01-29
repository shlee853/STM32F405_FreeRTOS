
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "debug.h"

#include "stm32fxxx.h"

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "config.h"
#include "cfassert.h"
#include "config.h"
#include "static_mem.h"


#define QUEUE_LENGTH 64
static xQueueHandle uartqueue;
STATIC_MEM_QUEUE_ALLOC(uartqueue, QUEUE_LENGTH, sizeof(uint8_t));

static bool isInit = false;
static bool hasOverrun = false;

#ifdef ENABLE_uart_DMA
static xSemaphoreHandle uartBusy;
static StaticSemaphore_t uartBusyBuffer;
static xSemaphoreHandle waitUntilSendDone;
static StaticSemaphore_t waitUntilSendDoneBuffer;
static DMA_InitTypeDef DMA_InitStructureShare;
static uint8_t dmaBuffer[64];
static bool    isUartDmaInitialized;
static uint32_t initialDMACount;
#endif


extern UART_HandleTypeDef huart6;

//int _write(int file, char* p, int len)
int UART_PRINTF(int file, char* p, int len)
{
		if(HAL_UART_Transmit(&huart6, p, len , 10 )!= 0)
			//		while(!LL_USART_IsActiveFlag_TXE(USART6));0
			//		usDelay(100);	// 문자 1개 출력당 약 100us 소요, Float, int형 차이 없음
			return -1;
		return len;
}


void uartInit(void) {
	  isInit = true;
}


void uartSendData(uint32_t size, uint8_t* data)
{
  uint32_t i;

  for(i = 0; i < size; i++)
  {
    while (!(USART6->SR & UART_FLAG_TXE));
    USART6->DR = (data[i] & 0x00FF);
  }
}


int uartPutchar(int ch)
{
    uartSendData(1, (uint8_t *)&ch);
    return (unsigned char)ch;
}


void uartGetchar(char * ch)
{
  xQueueReceive(uartqueue, ch, portMAX_DELAY);
}


uint32_t uartbytesAvailable()
{
  return uxQueueMessagesWaiting(uartqueue);
}

uint32_t uartQueueMaxLength()
{
  return QUEUE_LENGTH;
}

bool uartDidOverrun()
{
  bool result = hasOverrun;
  hasOverrun = false;

  return result;
}
