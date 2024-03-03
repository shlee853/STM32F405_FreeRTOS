/**

 * ICM20602.c
 * @author ChrisP @ M-HIVE

 * This library source code has been created for STM32F4. Only supports SPI.
 *
 * Development environment specifics:
 * STM32CubeIDE 1.0.0
 * STM32CubeF4 FW V1.24.1
 * STM32F4 LL Driver(SPI) and HAL Driver(RCC for HAL_Delay() function)
 *
 * Created by ChrisP(Wonyeob Park) @ M-HIVE Embedded Academy, July, 2019
 * Rev. 1.0
 *
 * https://github.com/ChrisWonyeobPark/
*/

/**
 * @brief ICM20602 structure definition.
 */

#include "ICM20602.h"
#include "debug.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_gpio.h"

extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;

Struct_ICM20602 ICM20602;
int32_t gyro_x_offset, gyro_y_offset, gyro_z_offset; // To remove offset


void ICM20602_GPIO_SPI_Initialization(void)
{
	
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
    HAL_GPIO_WritePin(GPIOA, SPI1_NSS_PIN_Pin, GPIO_PIN_RESET);
	

    __HAL_SPI_ENABLE(&hspi1);

	CHIP_DESELECT(ICM20602);
}


unsigned char SPI1_SendByte(unsigned char data)
{
	while(LL_SPI_IsActiveFlag_TXE(ICM20602_SPI_CHANNEL)==RESET);
	LL_SPI_TransmitData8(ICM20602_SPI_CHANNEL, data);
	
	while(LL_SPI_IsActiveFlag_RXNE(ICM20602_SPI_CHANNEL)==RESET);
	return LL_SPI_ReceiveData8(ICM20602_SPI_CHANNEL);
}

//////////////////////////////////////////////////////////////

uint8_t ICM20602_Readbyte(uint8_t reg_addr)
{
	uint8_t val;

	CHIP_SELECT(ICM20602);
	SPI1_SendByte(reg_addr | 0x80); //Register. MSB 1 is read instruction.
	val = SPI1_SendByte(0x00); //Send DUMMY to read data
	CHIP_DESELECT(ICM20602);
	
	return val;
}

void ICM20602_Readbytes(unsigned char reg_addr, unsigned char len, unsigned char* data)
{
	unsigned int i = 0;

	CHIP_SELECT(ICM20602);
	SPI1_SendByte(reg_addr | 0x80); //Register. MSB 1 is read instruction.
	while(i < len)
	{
		data[i++] = SPI1_SendByte(0x00); //Send DUMMY to read data
	}
	CHIP_DESELECT(ICM20602);
}

void ICM20602_Writebyte(uint8_t reg_addr, uint8_t val)
{
	CHIP_SELECT(ICM20602);
	SPI1_SendByte(reg_addr & 0x7F); //Register. MSB 0 is write instruction.
	SPI1_SendByte(val); //Send Data to write
	CHIP_DESELECT(ICM20602);
}

void ICM20602_Writebytes(unsigned char reg_addr, unsigned char len, unsigned char* data)
{
	unsigned int i = 0;
	CHIP_SELECT(ICM20602);
	SPI1_SendByte(reg_addr & 0x7F); //Register. MSB 0 is write instruction.
	while(i < len)
	{
		SPI1_SendByte(data[i++]); //Send Data to write
	}
	CHIP_DESELECT(ICM20602);
}


int ICM20602_Initialization(void)
{

	uint8_t who_am_i = 0;
	int16_t accel_raw_data[3] = {0};  // To remove offset
	int16_t gyro_raw_data[3] = {0};   // To remove offset
	
	ICM20602_GPIO_SPI_Initialization();
	
	DEBUG_PRINT("Checking ICM20602...");
	
	// check WHO_AM_I (0x75)
	who_am_i = ICM20602_Readbyte(WHO_AM_I); 

	// who am i = 0x12
	if(who_am_i == 0x12)
	{
		DEBUG_PRINT("\r\nICM20602 who_am_i = 0x%02x.................Pass\n", who_am_i);
	}
	// recheck
	else if(who_am_i != 0x12)
	{
		who_am_i = ICM20602_Readbyte(WHO_AM_I); // check again WHO_AM_I (0x75)

		if (who_am_i != 0x12){
			DEBUG_PRINT( "ICM20602 is not found: 0x%02x Should be 0x%02x\n", who_am_i, 0x12);
			return 1; //ERROR
		}
	}
	
	// Reset ICM20602
	// PWR_MGMT_1 0x6B
	ICM20602_Writebyte(PWR_MGMT_1, 0x80); //Reset ICM20602
	delay_us(5000000);

	// PWR_MGMT_1 0x6B
	ICM20602_Writebyte(PWR_MGMT_1, 0x01); // Enable Temperature sensor(bit4-0), Use PLL(bit2:0-01)
									// 온도센서 끄면 자이로 값 이상하게 출력됨
	delay_us(5000000);

	// PWR_MGMT_2 0x6C
	//ICM20602_Writebyte(PWR_MGMT_2, 0x38); // Disable Acc(bit5:3-111), Enable Gyro(bit2:0-000)
	ICM20602_Writebyte( PWR_MGMT_2, 0x00 ); // Enable Acc(bit5:3-000), Enable Gyro(bit2:0-000)
	delay_us(5000000);

	// set sample rate to 1000Hz and apply a software filter
	ICM20602_Writebyte(SMPLRT_DIV, 0x00);
	delay_us(5000000);
	
	// Gyro DLPF Config
//	ICM20602_Writebyte(CONFIG, 0x00); // Gyro LPF fc 250Hz(bit2:0-000)
	ICM20602_Writebyte(CONFIG, 0x05); // Gyro LPF fc 20Hz(bit2:0-100) at 1kHz sample rate
	delay_us(5000000);

	// GYRO_CONFIG 0x1B
	ICM20602_Writebyte(GYRO_CONFIG, 0x18); // Gyro sensitivity 2000 dps(bit4:3-11), FCHOICE (bit1:0-00)
	delay_us(5000000);

	// ACCEL_CONFIG 0x1C
	ICM20602_Writebyte(ACCEL_CONFIG, 0x18); // Acc sensitivity 16g
	delay_us(5000000);
	
	// ACCEL_CONFIG2 0x1D
	ICM20602_Writebyte(ACCEL_CONFIG2, 0x03); // Acc FCHOICE 1kHz(bit3-0), DLPF fc 44.8Hz(bit2:0-011)
	delay_us(5000000);
	
	// Enable Interrupts when data is ready
	ICM20602_Writebyte(INT_ENABLE, 0x01); // Enable DRDY Interrupt
	delay_us(5000000);
	
	//DEBUG_PRINT("gyro bias: %d %d %d\n", gyro_x_offset, gyro_y_offset, gyro_z_offset);
	
	// Remove Gyro X offset
//	ICM20602_Writebyte( XG_OFFS_USRH, offset_x>>8 );	// gyro x offset high byte
//	ICM20602_Writebyte( XG_OFFS_USRL, offset_x );	// gyro x offset low byte
//	
//	// Remove Gyro Y offset
//	ICM20602_Writebyte( YG_OFFS_USRH, offset_y>>8 );	// gyro y offset high byte
//	ICM20602_Writebyte( YG_OFFS_USRL, offset_y );	// gyro y offset low byte
//	
//	// Remove Gyro Z offset
//	ICM20602_Writebyte( ZG_OFFS_USRH, offset_z>>8 );	// gyro z offset high byte
//	ICM20602_Writebyte( ZG_OFFS_USRL, offset_z );	// gyro z offset low byte

	return 0; //OK
}

void ICM20602_Get6AxisRawData(short* accel, short* gyro)
{
	unsigned char data[14];
	ICM20602_Readbytes(ACCEL_XOUT_H, 14, data);
	
	accel[0] = (data[0] << 8) | data[1];
	accel[1] = (data[2] << 8) | data[3];
	accel[2] = (data[4] << 8) | data[5];

	gyro[0] = ((data[8] << 8) | data[9]);
	gyro[1] = ((data[10] << 8) | data[11]);
	gyro[2] = ((data[12] << 8) | data[13]);
}

void ICM20602_Get3AxisGyroRawData(short* gyro)
{
	unsigned char data[6];
	ICM20602_Readbytes(GYRO_XOUT_H, 6, data);
	
	gyro[0] = ((data[0] << 8) | data[1]);
	gyro[1] = ((data[2] << 8) | data[3]);
	gyro[2] = ((data[4] << 8) | data[5]);
}

void ICM20602_Get3AxisAccRawData(short* accel)
{
	unsigned char data[6];
	ICM20602_Readbytes(ACCEL_XOUT_H, 6, data);
	
	accel[0] = ((data[0] << 8) | data[1]);
	accel[1] = ((data[2] << 8) | data[3]);
	accel[2] = ((data[4] << 8) | data[5]);
}

int ICM20602_DataReady(void)
{
	return LL_GPIO_IsInputPinSet(ICM20602_INT_PORT, ICM20602_INT_PIN);
}
