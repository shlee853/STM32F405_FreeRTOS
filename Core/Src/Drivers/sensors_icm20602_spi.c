

#include <string.h>

#include "stm32fxxx.h"
#include "bstdr_types.h"

#include "ICM20602.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;

//#include "sensors_bmi088_common.h"

#define DUMMY_BYTE    0x00

/* Defines and buffers for full duplex SPI DMA transactions */
/* The buffers must not be placed in CCM */
#define SPI_MAX_DMA_TRANSACTION_SIZE    15
static uint8_t spiTxBuffer[SPI_MAX_DMA_TRANSACTION_SIZE + 1];
static uint8_t spiRxBuffer[SPI_MAX_DMA_TRANSACTION_SIZE + 1];
static xSemaphoreHandle spiTxDMAComplete;
StaticSemaphore_t spiTxDMACompleteBuffer;
static xSemaphoreHandle spiRxDMAComplete;
StaticSemaphore_t spiRxDMACompleteBuffer;


static bool isInit;

static char spiSendByte(char byte)
{
  char rx_data;

  /* Loop while DR register in not emplty */
  while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE) == RESET);

  /* Send byte through the SPI peripheral */
  HAL_SPI_Transmit(&hspi1, &byte, 1, 10);

  /* Wait to receive a byte */
  while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_RXNE) == RESET);

  /* Return the byte read from the SPI bus */

  HAL_SPI_Receive(&hspi1, &rx_data, 1, 10);

  return rx_data;
}

static char spiReceiveByte()
{
  return spiSendByte(DUMMY_BYTE);
}

static void spiDMATransaction(uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
  ASSERT(len < SPI_MAX_DMA_TRANSACTION_SIZE);

  // Disable peripheral before setting up for duplex DMA
  __HAL_SPI_ENABLE(&hspi1);

  // DMA already configured, just need to set memory addresses and read command byte
  spiTxBuffer[0] = reg_addr;
  hdma_spi1_tx.Instance->M0AR 	= (uint32_t)&spiTxBuffer[0];
  hdma_spi1_tx.Instance->NDTR 	= len + 1;


  hdma_spi1_tx.Instance->M0AR 	= (uint32_t)&spiRxBuffer[0];
  hdma_spi1_tx.Instance->NDTR 	= len + 1;


  // Enable SPI DMA Interrupts TX
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

  // Enable SPI DMA Interrupts RX
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);


  // Clear DMA Flags

  __HAL_DMA_CLEAR_FLAG(&hdma_spi1_tx, DMA_FLAG_FEIF3_7|DMA_FLAG_DMEIF3_7|DMA_FLAG_TEIF3_7|DMA_FLAG_HTIF3_7|DMA_FLAG_TCIF3_7);
  __HAL_DMA_CLEAR_FLAG(&hdma_spi1_rx, DMA_FLAG_FEIF0_4|DMA_FLAG_DMEIF0_4|DMA_FLAG_TEIF0_4|DMA_FLAG_HTIF0_4|DMA_FLAG_TCIF0_4);


  // Enable DMA Streams
  __HAL_DMA_ENABLE(&hdma_spi1_tx);
  __HAL_DMA_ENABLE(&hdma_spi1_rx);

  // Enable SPI DMA requests
//  SPI_I2S_DMACmd(ICM20602_SPI, SPI_I2S_DMAReq_Tx, ENABLE);
//  SPI_I2S_DMACmd(ICM20602_SPI, SPI_I2S_DMAReq_Rx, ENABLE);

  // Enable peripheral to begin the transaction
  __HAL_SPI_ENABLE(&hspi1);

  // Wait for completion
  // TODO: Better error handling rather than passing up invalid data
  xSemaphoreTake(spiTxDMAComplete, portMAX_DELAY);
  xSemaphoreTake(spiRxDMAComplete, portMAX_DELAY);

  // Copy the data (discarding the dummy byte) into the buffer
  // TODO: Avoid this memcpy either by figuring out how to configure the STM SPI to discard the byte or handle it higher up
  memcpy(reg_data, &spiRxBuffer[1], len);
}

static bstdr_ret_t spi_burst_read(uint8_t dev_id, uint8_t reg_addr,
                                  uint8_t *reg_data, uint16_t len)
{
  /**< Burst read code comes here */

  CHIP_SELECT();

  if (len <= 1 || len > SPI_MAX_DMA_TRANSACTION_SIZE)
  {
    spiSendByte(reg_addr);
    for (int i = 0; i < len; i++)
    {
      reg_data[i] = spiReceiveByte();
    }
  }
  else
  {
    spiDMATransaction(reg_addr, reg_data, len);
  }

  CHIP_DESELECT();

  return BSTDR_OK;
}

static bstdr_ret_t spi_burst_write(uint8_t dev_id, uint8_t reg_addr,
                                   uint8_t *reg_data, uint16_t len)
{

  CHIP_SELECT();

  spiSendByte(reg_addr);
  for (int i = 0; i < len; i++)
  {
    spiSendByte(reg_data[i]);
  }

  CHIP_DESELECT();

  return BSTDR_OK;
}

static void spiConfigure(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* Enable the SPI  */
  __HAL_SPI_ENABLE(&hspi1);

}

void sensorsIcm20602Init_SPI(void)
{

  if (isInit)
	{
	  return;
	}

  spiInit();

  isInit = true;
}


/* Initialisation */
void spiInit(void)
{

  if (isInit)
    return;

  isInit = true;

	GPIO_InitTypeDef GPIO_InitStruct;

	__HAL_RCC_SPI1_CLK_ENABLE();

	__HAL_RCC_GPIOA_CLK_ENABLE();
	/**SPI1 GPIO Configuration
	PA5   ------> SPI1_SCK
	PA6   ------> SPI1_MISO
	PA7   ------> SPI1_MOSI
	*/
  GPIO_InitStruct.Pin = SPI1_SCK_PIN_Pin|SPI1_MISO_PIN_Pin|SPI1_MOSI_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SPI1_NSS_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI1_NSS_PIN_GPIO_Port, &GPIO_InitStruct);


  GPIO_InitStruct.Pin = SPI1_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI1_INT_GPIO_Port, &GPIO_InitStruct);

  spiDMAInit();

  /* disable the chip select */
  CHIP_DESELECT();

  spiConfigure();
}

void spiDMAInit(void)
{


    hdma_spi1_rx.Instance = DMA2_Stream0;
    hdma_spi1_rx.Init.Channel = DMA_CHANNEL_3;
    hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_rx.Init.Mode = DMA_NORMAL;
    hdma_spi1_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi1_rx) != HAL_OK)
    {
      Error_Handler();
    }

	/**ICM20602 GPIO Control Configuration
	 * PC4  ------> ICM20602_SPI_CS_PIN (output)
	 * PC5  ------> ICM20602_INT_PIN (input)
	 */
	/**/


    __HAL_LINKDMA(&hspi1,hdmarx,hdma_spi1_rx);

    /* SPI1_TX Init */
    hdma_spi1_tx.Instance = DMA2_Stream3;
    hdma_spi1_tx.Init.Channel = DMA_CHANNEL_3;
    hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_tx.Init.Mode = DMA_NORMAL;
    hdma_spi1_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(&hspi1,hdmatx,hdma_spi1_tx);




  spiTxDMAComplete = xSemaphoreCreateBinaryStatic(&spiTxDMACompleteBuffer);
  spiRxDMAComplete = xSemaphoreCreateBinaryStatic(&spiRxDMACompleteBuffer);
}
/*
void sensorsICM20602_SPI_deviceInit(struct ICM20602_dev *device)
{
  spiInit();

//  device->accel_id = ICM20602_ACCEL_I2C_ADDR_PRIMARY;
//  device->gyro_id = ICM20602_GYRO_I2C_ADDR_PRIMARY;
//  device->interface = ICM20602_SPI_INTF;
  device->read = spi_burst_read;
  device->write = spi_burst_write;
//  device->delay_ms = ICM20602_ms_delay;
}

void __attribute__((used)) ICM20602_SPI_TX_DMA_IRQHandler(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // Stop and cleanup DMA stream
  DMA_ITConfig(ICM20602_SPI_TX_DMA_STREAM, DMA_IT_TC, DISABLE);
  DMA_ClearITPendingBit(ICM20602_SPI_TX_DMA_STREAM, ICM20602_SPI_TX_DMA_FLAG_TCIF);

  // Clear stream flags
  DMA_ClearFlag(ICM20602_SPI_TX_DMA_STREAM,ICM20602_SPI_TX_DMA_FLAG_TCIF);

  // Disable SPI DMA requests
  SPI_I2S_DMACmd(ICM20602_SPI, SPI_I2S_DMAReq_Tx, DISABLE);

  // Disable streams
  DMA_Cmd(ICM20602_SPI_TX_DMA_STREAM, DISABLE);

  // Give the semaphore, allowing the SPI transaction to complete
  xSemaphoreGiveFromISR(spiTxDMAComplete, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken)
  {
    portYIELD();
  }
}

void __attribute__((used)) ICM20602_SPI_RX_DMA_IRQHandler(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // Stop and cleanup DMA stream
  DMA_ITConfig(ICM20602_SPI_RX_DMA_STREAM, DMA_IT_TC, DISABLE);
  DMA_ClearITPendingBit(ICM20602_SPI_RX_DMA_STREAM, ICM20602_SPI_RX_DMA_FLAG_TCIF);

  // Clear stream flags
  DMA_ClearFlag(ICM20602_SPI_RX_DMA_STREAM, ICM20602_SPI_RX_DMA_FLAG_TCIF);

  // Disable SPI DMA requests
  SPI_I2S_DMACmd(ICM20602_SPI, SPI_I2S_DMAReq_Rx, DISABLE);

  // Disable streams
  DMA_Cmd(ICM20602_SPI_RX_DMA_STREAM, DISABLE);

  // Give the semaphore, allowing the SPI transaction to complete
  xSemaphoreGiveFromISR(spiRxDMAComplete, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken)
  {
    portYIELD();
  }
}


*/
