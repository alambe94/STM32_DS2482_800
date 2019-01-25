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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include"ssd1306.h"
#include "fonts.h"
#include "stdlib.h"
#include "string.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
RTC_TimeTypeDef RTC_time;

uint16_t discharge_time_seconds, discharge_time_minutes, discharge_time_hours;
uint16_t clock_time_seconds_prev;
uint16_t clock_time_seconds, clock_time_minutes, clock_time_hours;

char seconds_s[5], minutes_s[5], hours_s[5];




//FUNCTION COMMANDS

/*********************************************/
//Device Reset (Parameter 0 byte)
#define  DEVICE_RESET_CMD               (0xF0)
/*********************************************/



/*********************************************/
//Set Read Pointer cammand
#define  SET_READ_POINTER_CMD           (0xE1)

//Set Read Pointer valid Command Parameter
#define  STATUS_REGISTER_PTR            (0xF0)
#define  READ_DATA_REGISTER_PTR         (0xE1)
#define  CHANNEL_SELECTION_REGISTER_PTR (0xD2)
#define  CONFIGURATION_REGISTER_PTR     (0xC3)
/*********************************************/

/*********************************************/
//Write Configuration (Parameter 1 byte)
#define  WRITE_CONFIGURATION_CMD        (0xD2)
/*********************************************/

/*********************************************/
//Channel Select (Parameter 1 byte)
#define  CHANNEL_SELECT_CMD             (0XC3)

#define  CHANNEL_0_CODE             (0XF0)
#define  CHANNEL_1_CODE             (0XE1)
#define  CHANNEL_2_CODE             (0XD2)
#define  CHANNEL_3_CODE             (0XC3)
#define  CHANNEL_4_CODE             (0XB4)
#define  CHANNEL_5_CODE             (0XA5)
#define  CHANNEL_6_CODE             (0X96)
#define  CHANNEL_7_CODE             (0X87)

#define  CHANNEL_0_RETURN           (0XB8)
#define  CHANNEL_1_RETURN           (0XB1)
#define  CHANNEL_2_RETURN           (0XAA)
#define  CHANNEL_3_RETURN           (0XA3)
#define  CHANNEL_4_RETURN           (0X9C)
#define  CHANNEL_5_RETURN           (0X95)
#define  CHANNEL_6_RETURN           (0X8E)
#define  CHANNEL_7_RETURN           (0X87)
/*********************************************/


//1-Wire Reset (Parameter 0 byte)
#define  OW_RESET_CMD               (0XB4)

//1-Wire Single Bit (Parameter 1 byte (only MSB))
#define  OW_SINGLE_BIT_CMD          (0X87)

//1-Wire Write Byte (Parameter 1 byte)
#define  OW_WRITE_BYTE_CMD          (0XA5)

//1-Wire Read Byte (Parameter 0 byte)
#define  OW_READ_BYTE_CMD           (0X96)

//1-Wire Single Bit (Parameter 1 byte (only MSB))
//1-Wire Triplet
#define  OW_TRIPLET_CMD             (0X78)


#define convert_T		0x44
#define read_scratchpad         0xBE
#define write_scratchpad	0x4E
#define copy_scratchpad         0x48
#define recall_E2		0xB8
#define read_power_supply	0xB4
#define skip_ROM		0xCC

#define resolution		12


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
HAL_StatusTypeDef DS2482_Write_Two_Byte(uint8_t cmd,uint8_t param)
    {
    uint8_t CMD_DS[2] = {cmd,param};
    return HAL_I2C_Master_Transmit(&hi2c1, 0x18<<1, CMD_DS, 2, 100);
    }

HAL_StatusTypeDef DS2482_Write_One_Byte(uint8_t cmd)
    {
    uint8_t CMD_DS[1] = {cmd};
    return HAL_I2C_Master_Transmit(&hi2c1, 0x18<<1, CMD_DS, 1, 100);
    }

HAL_StatusTypeDef DS2482_Read_Byte(uint8_t* data)
    {
    return HAL_I2C_Master_Receive(&hi2c1, 0x18<<1, data, 1, 100);
    }

void DS2482_Test()
    {
    DS2482_Write_One_Byte(DEVICE_RESET_CMD);
    DS2482_Write_One_Byte(OW_RESET_CMD);
    DS2482_Write_One_Byte(OW_READ_BYTE_CMD);
    DS2482_Write_Two_Byte(SET_READ_POINTER_CMD,STATUS_REGISTER_PTR);
    DS2482_Write_Two_Byte(SET_READ_POINTER_CMD,READ_DATA_REGISTER_PTR);
    DS2482_Write_Two_Byte(SET_READ_POINTER_CMD,CHANNEL_SELECTION_REGISTER_PTR);
    DS2482_Write_Two_Byte(SET_READ_POINTER_CMD,CONFIGURATION_REGISTER_PTR);

    DS2482_Write_Two_Byte(WRITE_CONFIGURATION_CMD,0xF0);

    DS2482_Write_Two_Byte(CHANNEL_SELECT_CMD,CHANNEL_0_CODE);
    DS2482_Write_Two_Byte(CHANNEL_SELECT_CMD,CHANNEL_1_CODE);
    DS2482_Write_Two_Byte(CHANNEL_SELECT_CMD,CHANNEL_2_CODE);
    DS2482_Write_Two_Byte(CHANNEL_SELECT_CMD,CHANNEL_3_CODE);
    DS2482_Write_Two_Byte(CHANNEL_SELECT_CMD,CHANNEL_4_CODE);
    DS2482_Write_Two_Byte(CHANNEL_SELECT_CMD,CHANNEL_5_CODE);
    DS2482_Write_Two_Byte(CHANNEL_SELECT_CMD,CHANNEL_6_CODE);
    DS2482_Write_Two_Byte(CHANNEL_SELECT_CMD,CHANNEL_7_CODE);

    DS2482_Write_Two_Byte(OW_SINGLE_BIT_CMD,0x80);

    DS2482_Write_Two_Byte(OW_WRITE_BYTE_CMD,0xA5);

    DS2482_Write_Two_Byte(OW_TRIPLET_CMD,0x80);

    uint8_t temp;

    DS2482_Write_Two_Byte(CHANNEL_SELECT_CMD,CHANNEL_1_CODE);
    DS2482_Write_Two_Byte(SET_READ_POINTER_CMD,CONFIGURATION_REGISTER_PTR);
    DS2482_Read_Byte(&temp);
if (temp == 0xF0)
    {
	ssd1306_Fill(Black);
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString("OK", Font_11x18, White);
	ssd1306_UpdateScreen();
    }
else
    {
	ssd1306_Fill(Black);
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString("fail", Font_11x18, White);
	ssd1306_UpdateScreen();
    }

    }

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
  MX_I2C1_Init();
  MX_RTC_Init();

  /* USER CODE BEGIN 2 */
  ssd1306_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	    uint8_t temp1;
	    uint8_t temp2;
	    float temp;
	    char temp_s[10];


	    DS2482_Write_One_Byte(OW_RESET_CMD);
	    HAL_Delay(1);
	    DS2482_Write_Two_Byte(OW_WRITE_BYTE_CMD,skip_ROM);
	    HAL_Delay(1);
	    DS2482_Write_Two_Byte(OW_WRITE_BYTE_CMD,convert_T);
	    HAL_Delay(1000);

	    DS2482_Write_One_Byte(OW_RESET_CMD);
	    HAL_Delay(1);
	    DS2482_Write_Two_Byte(OW_WRITE_BYTE_CMD,skip_ROM);
	    HAL_Delay(1);
	    DS2482_Write_Two_Byte(OW_WRITE_BYTE_CMD,read_scratchpad);
	    HAL_Delay(1);
	    DS2482_Write_One_Byte(OW_READ_BYTE_CMD);
	    HAL_Delay(1);
	    DS2482_Write_Two_Byte(SET_READ_POINTER_CMD,READ_DATA_REGISTER_PTR);
	    DS2482_Read_Byte(&temp1);
	    DS2482_Write_One_Byte(OW_READ_BYTE_CMD);
	    HAL_Delay(1);
	    DS2482_Write_Two_Byte(SET_READ_POINTER_CMD,READ_DATA_REGISTER_PTR);
	    DS2482_Read_Byte(&temp2);

	    temp = temp2;
	    temp *= 256.0;
	    temp += temp1;
	    temp *= 0.0625;

	    ssd1306_SetCursor(0, 0);
	    sprintf(temp_s,"%0.2f" , temp);
	    ssd1306_WriteString(temp_s, Font_11x18, White);
	    ssd1306_UpdateScreen();






	}
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

/* I2C1 init function */

static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef DateToUpdate;

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initialize RTC and set the Time and Date 
    */
  if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x32F2){
  sTime.Hours = 4;
  sTime.Minutes = 51;
  sTime.Seconds = 0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_MARCH;
  DateToUpdate.Date = 5;
  DateToUpdate.Year = 18;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR1,0x32F2);
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OLED_VCC_GPIO_Port, OLED_VCC_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OLED_GND_GPIO_Port, OLED_GND_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_STATUS_Pin */
  GPIO_InitStruct.Pin = LED_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(LED_STATUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_VCC_Pin OLED_GND_Pin */
  GPIO_InitStruct.Pin = OLED_VCC_Pin|OLED_GND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
	while (1) {
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
