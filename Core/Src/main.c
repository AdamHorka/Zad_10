/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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
// Space for your global variables

//uint8_t tx_data[] = "I"; 			// odosielane data
uint8_t rx_data[20];	 			// prijimane data
uint8_t start = 0;					// zaciatok citania retazca
uint8_t pwm_array[2] = {'0','0'};	// pomocne pole
uint8_t pwm_int = 0;				// hodnota PWM
uint8_t mode = 100; 				// 0 pre manual, 1 pre auto, 2 pre pwm

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Function processing DMA Rx data
void receive_dma_data(const uint8_t* data, uint16_t len);
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

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  USART2_RegisterCallback(receive_dma_data);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	#if POLLING
			//Polling for new data, no interrupts
			USART2_CheckDmaReception();
			LL_mDelay(10);
	#else
		USART2_PutBuffer(tx_data, sizeof(tx_data));
		LL_mDelay(1000);
	#endif

	//USART2_PutBuffer(tx_data, sizeof(tx_data));
	LL_mDelay(100);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  /* USER CODE BEGIN 4 */
  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
    {
    Error_Handler();
    }
    LL_RCC_HSI_Enable();
  /* USER CODE END 4 */
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_Init1msTick(8000000);
  /* USER CODE BEGIN 5 */
  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
  /* USER CODE END 5 */
  LL_SetSystemCoreClock(8000000);
}

/* USER CODE BEGIN 6 */

/* USER CODE END 6 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */

/* USER CODE BEGIN 7 */
void receive_dma_data(const uint8_t* data, uint16_t len)
{
	if (start == 0) //znaci ze sme este nenacali citanie znakom $
	{
		for(uint8_t i = 0; i<len; i++)
		{
			if(*(data+i)=='$' && start == 0) //prijali sme $
			{
				start = 1;
			}

			if(start == 1 && *(data+i)!='$') //prijali sme znak $ a zaciname citat ale $ nezapisujeme
			{
				rx_data[i-1] = *(data+i);
			}

			if(i>0 && start == 1 && *(data+i)=='$')
			{
				start = 0;

				// Rozlisujeme aky retazec sme prijali
				if(strcmp(rx_data, "manual") == 0)
				{
					mode = 0;
					memset(rx_data,'\0',10); //zmazeme retazec z arrayu
				}
				if(strcmp(rx_data, "auto") == 0)
				{
					mode = 1;
					memset(rx_data,'\0',10); //zmazeme retazec z arrayu
				}
				//tato podmienka je odporna, prerobit
				if(mode == 0 && rx_data[0]=='P' && rx_data[1]=='W' && rx_data[2]=='M' &&  (rx_data[3]>= '0' && rx_data[3]<= '9') && (rx_data[4]>= '0' && rx_data[4]<= '9')) //mozeme sa prepnut len ak sme v manual mode
				{
					mode = 2;

					pwm_array[0] = rx_data[3];
					pwm_array[1] = rx_data[4];

					pwm_int = atoi(pwm_array); //transformujeme hodnoty arrayu pwm_array na integer pwm pre dalsie pouzitie

					memset(rx_data,'\0',10); //zmazeme retazec z arrayu
				}

			}
		}
	}
}
/* USER CODE END 7 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
