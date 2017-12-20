/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "lcd.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define SHT20		0x80	// 0x40<<1
#define SHT20_T		0xf3
#define SHT20_RH	0xf5
int tValue;
int rhValue;
char tDisp[10];	// ex) -10.0
char rhDisp[10];	// ex) 25
int dispIdx;
uint8_t txBuf[3];
uint8_t rxBuf[3];

#define BH1750	 0x46 // 0x23<<1
#define BH1750_H	0x20	// H-resolution, one-time
char lDisp[10];
int lValue;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_I2C2_Init();

  /* USER CODE BEGIN 2 */
  LCD_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  txBuf[0] = SHT20_T;
	  HAL_I2C_Master_Transmit(&hi2c2, SHT20, txBuf, 1, 1000);
	  HAL_Delay(500);
	  HAL_I2C_Master_Receive(&hi2c2, SHT20, rxBuf, 3, 1000);
	  tValue = (rxBuf[0]<<8)|rxBuf[1];
	  tValue = (tValue*175.72/65536-46.85)*10;
	  dispIdx = 0;
	  if(tValue<0)
	  {
		  tDisp[dispIdx++] = '-';
		  tValue = -tValue;	// 양수로 변경
	  }
	  if(tValue>=100) // 10.0도 이상이면..
	  {
		  tDisp[dispIdx++] = tValue/100 + '0'; // 10도 단위 온도
	  }
	  tDisp[dispIdx++] = tValue%100/10 + '0'; // 1도
	  tDisp[dispIdx++] = '.';
	  tDisp[dispIdx++] = tValue%10 + '0';
	  tDisp[dispIdx++] = 0xdf;	// 도
	  tDisp[dispIdx++] = 'C';	// C
	  tDisp[dispIdx] = '\0';
	  LCD_setCursor(0, 0);
	  LCD_print("        ");
	  LCD_setCursor(0, 0);
	  LCD_print(tDisp);
	  HAL_Delay(1000);

	  txBuf[0] = SHT20_RH;
	  HAL_I2C_Master_Transmit(&hi2c2, SHT20, txBuf, 1, 1000);
	  HAL_Delay(500);
	  HAL_I2C_Master_Receive(&hi2c2, SHT20, rxBuf, 3, 1000);
	  rhValue = (rxBuf[0]<<8)|rxBuf[1];
	  rhValue = (rhValue*125.0/65536-6.0);
	  dispIdx = 0;
	  if(rhValue>=10) // 10% 이상
	  {
		  rhDisp[dispIdx++] = rhValue/10 + '0'; // 10도 단위 온도
	  }
	  rhDisp[dispIdx++] = rhValue%10 + '0'; // 1도
	  rhDisp[dispIdx++] = '%';
	  rhDisp[dispIdx] = '\0';

	  LCD_setCursor(0, 1);
	  LCD_print("        ");
	  LCD_setCursor(0, 1);
	  LCD_print(rhDisp);
	  HAL_Delay(1000);

	  txBuf[0] = BH1750_H;
	  HAL_I2C_Master_Transmit(&hi2c2, BH1750, txBuf, 1, 1000);
	  HAL_Delay(300);
	  HAL_I2C_Master_Receive(&hi2c2, BH1750, rxBuf, 2, 1000);
	  lValue = (rxBuf[0]<<8)|rxBuf[1];
	  lValue = (lValue/1.2);
	  dispIdx = 0;
	  if(lValue>=10000) // 10000
	  {
		  lDisp[dispIdx++] = lValue/10000+'0'; // 10도 단위 온도
	  }
	  if(dispIdx!=0)
		  lDisp[dispIdx++] = lValue%10000/1000 + '0';
	  else if(lValue>=1000)
		  lDisp[dispIdx++] = lValue/1000 + '0';

	  if(dispIdx!=0)
		  lDisp[dispIdx++] = lValue%1000/100 + '0';
	  else if(lValue>=100)
		  lDisp[dispIdx++] = lValue/100 + '0';

	  if(dispIdx!=0)
		  lDisp[dispIdx++] = lValue%100/10 + '0';
	  else if(lValue>=10)
		  lDisp[dispIdx++] = lValue/10 + '0';

	  lDisp[dispIdx++] = lValue%10 + '0';
	  lDisp[dispIdx++] = 'L';
	  lDisp[dispIdx++] = 'x';
	  lDisp[dispIdx] = '\0';
	  LCD_setCursor(8, 1);
	  LCD_print("        ");
	  LCD_setCursor(8, 1);
	  LCD_print(lDisp);
	  HAL_Delay(1000);

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_8 
                          |GPIO_PIN_9|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 PC8 
                           PC9 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_8 
                          |GPIO_PIN_9|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
