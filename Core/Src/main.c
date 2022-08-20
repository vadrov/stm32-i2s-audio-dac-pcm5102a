/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  * Подключение аудио-ЦАП PCM5102A к микроконтроллеру STM32F401CCU6 по I2S с DMA. 
  * Без HAL (на CMSIS и LL)
  *
  * Автор: VadRov
  * https://www.youtube.com/c/VadRov
  * https://zen.yandex.ru/vadrov
  * https://vk.com/vadrov
  * https://t.me/vadrov_channel
  *
  * Схема подключения PCM5102 к плате микроконтроллера в файле "подключение PCM5102A к МК по I2S.png"
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//инициализация I2S3
static void I2S3_Init(void)
{
	LL_I2S_InitTypeDef I2S_InitStruct = {0};
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	//включаем тактирование spi3/i2s3
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);
	//включаем тактирование порта B
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	//включаем тактирование порта A
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

	//настраиваем модуль PLLI2S
	LL_RCC_PLLI2S_ConfigDomain_I2S(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLI2SM_DIV_25, 302, LL_RCC_PLLI2SR_DIV_2);

	//включаем модуль PLLI2S
	RCC->CR |= RCC_CR_PLLI2SON;

	//ждем активации модуля PLLI2S
	while (!(RCC->CR & RCC_CR_PLLI2SRDY)) { ; }

	//включаем тактирование I2S от модуля PLLI2S
	RCC->CFGR &= ~RCC_CFGR_I2SSRC;

	/* I2S3 конфигурация GPIO (для режима transmit master):
	  PB3   ------> I2S3_CK (AF06)
	  PB5   ------> I2S3_SD (AF06)
	  PA15	------> I2S3_WS (AF06)
	*/
	//выводы PB3 и PB5
	GPIO_InitStruct.Pin = LL_GPIO_PIN_3|LL_GPIO_PIN_5;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	//вывод PA15
	GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	//включаем тактирование контроллера DMA1
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	//настраиваем приоритет прерываний потока
	NVIC_SetPriority(DMA1_Stream5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	//включаем прерывания от потока
	NVIC_EnableIRQ(DMA1_Stream5_IRQn);

	//настройка DMA
	LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_5, LL_DMA_CHANNEL_0);
	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_5, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_5, LL_DMA_PRIORITY_LOW);
	LL_DMA_SetMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MODE_CIRCULAR);
	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_5, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_5, LL_DMA_PDATAALIGN_HALFWORD);
	LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_5, LL_DMA_MDATAALIGN_HALFWORD);
	LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_5);

	//настройка I2S3
	I2S_InitStruct.Mode = LL_I2S_MODE_MASTER_TX;
	I2S_InitStruct.Standard = LL_I2S_STANDARD_PHILIPS;
	I2S_InitStruct.MCLKOutput = LL_I2S_MCLK_OUTPUT_DISABLE;
	I2S_InitStruct.DataFormat = LL_I2S_DATAFORMAT_16B;
	I2S_InitStruct.ClockPolarity = LL_I2S_POLARITY_LOW;
	I2S_InitStruct.AudioFreq = LL_I2S_AUDIOFREQ_44K;
	LL_I2S_Init(SPI3, &I2S_InitStruct);
}

void I2S_StartTransmitDMA(SPI_TypeDef *spi, DMA_TypeDef *dma_x, uint32_t stream, uint32_t *buff, uint32_t buff_size)
{
	//выбираем режим i2s и включаем i2s
	spi->I2SCFGR |= (SPI_I2SCFGR_I2SMOD | SPI_I2SCFGR_I2SE);
	DMA_Stream_TypeDef *dma_TX = ((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)dma_x + STREAM_OFFSET_TAB[stream])));
	uint8_t shift[8] = {0, 6, 16, 22, 0, 6, 16, 22}; //битовое смещение во флаговых регистрах IFCR (L и H)
	volatile uint32_t *ifcr_tx = (stream > 3) ? &(dma_x->HIFCR) : &(dma_x->LIFCR);
	//выключаем канал DMA передачи tx
	dma_TX->CR &= ~DMA_SxCR_EN;
	while (dma_TX->CR & DMA_SxCR_EN) {__NOP();} //ждем отключения канала DMA
	//сбрасываем флаги прерываний tx
	*ifcr_tx = 0x3F<<shift[stream];
	//разрешаем spi принимать запросы от DMA
	spi->CR2 |= SPI_CR2_TXDMAEN;
	//запрещаем прерывания по некоторым событиям канала передачи tx и режим двойного буфера
	dma_TX->CR &= ~(DMA_SxCR_DMEIE | DMA_SxCR_TEIE | DMA_SxCR_DBM);
	dma_TX->FCR &= ~DMA_SxFCR_FEIE;
	//разрешаем прерывания по окончанию передачи и по передаче половины буфера
	dma_TX->CR |= (DMA_SxCR_TCIE | DMA_SxCR_HTIE);
	//настраиваем адреса и длину
	dma_TX->PAR = (uint32_t)(&spi->DR); //приемник периферия - адрес регистра DR spi
	dma_TX->M0AR = (uint32_t)buff; 		//источник память - адрес буфера исходящих данных
	dma_TX->NDTR = (uint32_t)buff_size; //размер передаваемых данных
	dma_TX->CR |= (DMA_SxCR_EN); //включение канала передачи
}

#define fabs_(x)	(x < 0 ? -x : x)

//таблица синусов от 0 до 90 градусов (значения умножены на 32768)
const uint16_t sin_tbl_int[91] = {	    0,   572,  1144,  1715,  2286,  2856,  3425,  3993,  4560,  5126,
									 5690,  6252,  6813,  7371,  7927,  8481,  9032,  9580, 10126, 10668,
									11207, 11743, 12275, 12803, 13328, 13848, 14365, 14876, 15384, 15886,
									16384, 16877, 17364, 17847, 18324, 18795, 19261, 19720, 20174, 20622,
									21063, 21498, 21926, 22348, 22763, 23170, 23571, 23965, 24351, 24730,
									25102, 25466, 25822, 26170, 26510, 26842, 27166, 27482, 27789, 28088,
									28378, 28660, 28932, 29197, 29452, 29698, 29935, 30163, 30382, 30592,
									30792, 30983, 31164, 31336, 31499, 31651, 31795, 31928, 32052, 32166,
									32270, 32365, 32449, 32524, 32588, 32643, 32688, 32723, 32748, 32763,
									32768 };

//возвращает синус угла умноженный на 32768.
//angle - угол в градусах
static int sin_int(int angle)
{
	if (fabs_(angle) > 360) angle %= 360;
	if (angle < 0) angle += 360;
	if (angle >=   0 && angle <=  90) return (int)sin_tbl_int[angle];
	if (angle >=  91 && angle <= 180) return (int)sin_tbl_int[180 - angle];
	if (angle >= 181 && angle <= 270) return -(int)sin_tbl_int[angle - 181];
	return -(int)sin_tbl_int[360 - angle]; //271...360
}

//возвращает косинус угла умноженный на 32768.
//angle - угол в градусах
static int cos_int(int angle)
{
	return sin_int(angle + 90);
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  I2S3_Init();

  int16_t buff[2*360];

  for(int i = 0; i < 2*360; i+=2)
  {
	  buff[i] =   (32767*sin_int(i>>1))>>15;
	  buff[i+1] = (32767*cos_int(i>>1))>>15;
  }

  I2S_StartTransmitDMA(SPI3, DMA1, LL_DMA_STREAM_5, (uint32_t*)buff, 2*360);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE2);
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_HSE_EnableCSS();
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_25, 168, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(84000000);
  LL_SetSystemCoreClock(84000000);
  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
