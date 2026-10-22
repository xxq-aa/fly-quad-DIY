/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "joy.h"
#include "key.h"
#include "NRF24L01.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int32_t freq = 0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint32_t Duty;
uint32_t capture;
uint8_t  Key;
uint8_t SendFlag = 0;
uint8_t Flag;

/*void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef*htim)
{
	if(htim->Instance == TIM3)
	{

	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		capture = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1)+1;
		freq = 1000000/capture;
	}
	else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
		uint32_t capture2 = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2)+1;
		Duty = capture2*100/capture;
		}
	}
}*/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_value, 4);
  Key_Init();
  NRF24L01_Init();
/* HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);*/

//  OLED_ShowString(3,1,"Buf[1]:     ");
//  OLED_ShowString(4,1,"Buf[2]:     ");
//  OLED_Clear();
 // OLED_ShowString(1,1,"Freq:000000Hz");
  //OLED_ShowString(2,1,"Duty:000%");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  JOY();
//	  OLED_ShowNum(1,1, adc_value[0], 4);
//	  OLED_ShowNum(2,1, adc_value[1], 4);



  int16_t KeyNum = Key_GetNum();

         if(KeyNum)
         {

        	 Key = KeyNum;
         }

         if(Key == 3)
         {
        	 Flag = !Flag;
        	 OLED_Clear();
         }

	NRF24L01_TxPacket[0] = Flag;
	NRF24L01_TxPacket[1] = js.x;
	NRF24L01_TxPacket[2] = js.y;
	NRF24L01_TxPacket[3] = js.zx;
	NRF24L01_TxPacket[4] = js.zy;
	NRF24L01_TxPacket[5] = Key;
	SendFlag = NRF24L01_Send();
	Key = 0;


	if(Flag == 0)
	{
		 OLED_ShowString(1,1,"x:");
		  OLED_ShowString(2,1,"y:");
		  OLED_ShowString(3,1,"zx:");
		  OLED_ShowString(4,1,"zy:");
		  OLED_ShowSignedNum(1,5,js1.x,4);
		 OLED_ShowSignedNum(2,5,js1.y,4);
		 OLED_ShowSignedNum(3,5,js1.zx,4);
		 OLED_ShowSignedNum(4,5,js1.zy,4);
	}
	else
		{
		if(NRF24L01_Receive() == 1)
				{
			if(NRF24L01_RxPacket[0] == 0x02)
			{
				float p_angle =*(float *)&NRF24L01_RxPacket[1];
				float r_angle =*( float *)&NRF24L01_RxPacket[5];
				float y_angle =*( float *)&NRF24L01_RxPacket[9];
				float high =*( float *)&NRF24L01_RxPacket[13];
				OLED_ShowString(1,1,"pitch");
				OLED_ShowString(2,1,"Roll ");
				OLED_ShowString(3,1,"yaw  ");
				OLED_ShowString(4,1,"High ");

			    	OLED_ShowSignedNum(1,8,p_angle,3);
				    OLED_ShowSignedNum(2,8,r_angle,3);
					OLED_ShowSignedNum(3,8,y_angle,3);
					OLED_ShowSignedNum(4,8,high*100,6);

				}
				}










         }



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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
#ifdef USE_FULL_ASSERT
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
