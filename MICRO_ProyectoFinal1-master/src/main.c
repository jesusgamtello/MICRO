/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "rgb_lcd.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#define THRESHOLD_TEMPERATURE 20
#define THRESHOLD_WIND 90
#define THRESHOLD_NOISE 60

#define MAX_ANGLE 1024
#define RGB (0x62<<1)
#define LCD (0x3E<<1)
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

int rain=0;
int onda=1 , status = 0 ;
long val1,val2,valortotal;
uint32_t adc[4], buffer[4];
int temperature,brightness,sound,speed;
float vsense = 5/4095;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//codigo para el lcd
unsigned char _displayfunction;
unsigned char _displaycontrol;
 

 
/*This function is used to turn on the display on the LCD.  It sends the 
hexadecimal code for the display on setting to the device in order to 
turn on the display*/
 
void displayOn() 
{
    _displaycontrol |= LCD_DISPLAYON;
    sendCommand(LCD_DISPLAYCONTROL | _displaycontrol);
}
 
/*This function is used to clear display on the LCD.  It sends the 
hexadecimal code for the clear instruction to the device in order to 
clear the display*/
void clear()
{
    sendCommand(LCD_CLEARDISPLAY);        
    HAL_Delay(20);        
}
 
 
/*This function is used to set the backlight color of the dispaly.  
It writes the provided r, g, and b values to the registers for the red
value, the green value, and the blue value respectively.*/
void setRGB(unsigned char r, unsigned char g, unsigned char b)
{
   
    setReg(REG_RED, r);
    setReg(REG_GREEN, g);
    setReg(REG_BLUE, b); 
    
}
 
/*This function is used to write to one of the registers for the backlight
of the LCD display.  The function takes in the address of which register to
write to and the value to be written to that register and then sends it to the
LCD display via the mbed I2C library*/
void setReg(unsigned char addr,unsigned char val)
{
    char data[2];
    data[0] = addr;
    data[1] = val;
    HAL_I2C_Master_Transmit(&hi2c1,RGB,&data,sizeof(data),100);
}
 
/*This function is used to write to the LCD screen.  It takes in a string of
characters and writes them to the 0x40 register of the display.*/
void print(char *str)
{   
    char data[2];
    data[0] = 0x40;
    while(*str)
    {
      data[1] = *str;
      HAL_I2C_Master_Transmit(&hi2c1,LCD,&data,sizeof(data),100);
      str++;
            
    }
 
 
}
 
 
/*This function sets where on the screen the text will be written next.  It 
takes in two values which indicate the row and column on the display that 
the cursor should be moved to*/
void locate(char col, char row)
{
    if(row == 0)
    {
        col = col | 0x80;
    }
    else
    {   
        col = col | 0xc0;
    }
 
    char data[2];
    data[0] = 0x80;
    data[1] = col;
    HAL_I2C_Master_Transmit(&hi2c1,LCD,&data,sizeof(data),100);
}
 
/*This function sends an instruction to the LCD display using the
 built in instruction codes from the device  datasheet using the mbed
 I2C library*/
void sendCommand(char value)
{
    char data[2] = {0x80, value};
    HAL_I2C_Master_Transmit(&hi2c1,LCD,&data,sizeof(data),100);
}
 
void init() 
{   
    //Initialize displayfunction parameter for setting up LCD display
   _displayfunction |= LCD_2LINE;
   _displayfunction |= LCD_5x10DOTS;
 
   //Wait for more than 30 ms after power rises above 4.5V per the data sheet
    HAL_Delay(5);
 
 
    // Send first function set command. Wait longer that 39 us per the data sheet
    sendCommand(LCD_FUNCTIONSET | _displayfunction);
    HAL_Delay(5);  
    
    // turn the display on
    displayOn();
 
    // clear the display
    clear();
    
    // Initialize backlight
    setReg(0, 0);
    setReg(1, 0);
    setReg(0x08, 0xAA);   
    
 
 
    
 
}
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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  val2=htim3.Init.Period;

 
  init();
  setRGB(255,255,255);


  encederLeds(status); // transmitimos al lcd
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  
    /* USER CODE END WHILE */
    HAL_ADC_Start_DMA(&hadc1, buffer, 4);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
    printf("ADC_VALUE es %d\n\r", adc[2]);
    printf("val1 %d\n\r", val1);
    printf("val2 %d\n\r", val2);
    printf("El valortotal es %d\n",valortotal);
    printf("Luminosidad %d\n\r", brightness);
    
    printf("Temperatura %d\n\r", temperature);
    printf("Sonido %d\n\r", sound);
    printf("Velocidad %d \n\r", speed);
    printf("\n");
    clear();
    imprimir_Lcd();
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    HAL_Delay(5000);
    
    

    status=next_state();
    encederLeds(status);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}


/*Esta función se utiliza para mostrar todas los datos 
en el LCD, de 16x2*/
void imprimir_Lcd(){
  init();
  volatile char v[7] = "xV:";
  volatile char t[5]="xT:";
  volatile char s[7]="xS:";
  volatile char l[9]="xL:";
 
  char array[3];
  char arrayT[2];
  char arrayS[3];
  char arrayL[3];
  int numerosV=2;
  int numerosT= 0;
  int numerosS= 2;
  int numerosL=4;
  if (speed<10)
  numerosV=3;
  if (speed>99)
  numerosV = 1;
  if (temperature>=0 && temperature<10)
  numerosT=1;
  if (sound<10)
  numerosS=3;
  if (sound>99)
  numerosS = 1;
  if (brightness<10)
  numerosL=5;
  if (brightness>99)
  numerosL= 3;
  if (brightness>999)
  numerosL=2;
  if (brightness>9999)
  numerosL=1;
  locate(0,0);
  strcat(v, itoa(speed, array, 10));
  HAL_I2C_Master_Transmit(&hi2c1,LCD,v,sizeof(v)-numerosV,100);
  locate(6,0);
  strcat(t, itoa(temperature, arrayT, 10));
  HAL_I2C_Master_Transmit(&hi2c1,LCD,t,sizeof(t)-numerosT,100);
  locate(11,0);
  strcat(s, itoa(sound, arrayT, 10));
  HAL_I2C_Master_Transmit(&hi2c1,LCD,s,sizeof(s)-numerosS,100);
  locate(0,1);
  strcat(l, itoa(brightness, arrayL, 10));
  HAL_I2C_Master_Transmit(&hi2c1,LCD,l,sizeof(l)-numerosL,100);
  locate(9,1);
  strcat(l, itoa(brightness, arrayL, 10));
  if (rain!=1){
  volatile char p[9]="xLluv:No";
  HAL_I2C_Master_Transmit(&hi2c1,LCD,p,sizeof(p)-1,100);
  }
  else{
  volatile char p[9]="xLluv:Si";
  HAL_I2C_Master_Transmit(&hi2c1,LCD,p,sizeof(p)-1,100);
  rain=0;
  }
  

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1119;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  //htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 75;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1119;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  //htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA7 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin==GPIO_PIN_10){
    rain=1;
    printf("rain %i\n",rain);
    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
    
  }
  /* Reseteando al estado Inicial*/ 
  if (GPIO_Pin==GPIO_PIN_9){

    encederLeds(0);

  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == hadc1.Instance)
  {
    for (int i = 0; i < 4; i++)
    {
      adc[i] = buffer[i];
      if (i == 0)
      {
        calcular_viento(i);
      }
      else if (i == 1)
      {
        calcular_sonido(i);
      }
      else if (i == 2)
      {
        calcular_luminosidad(i);
      }
      else if (i == 3)
      {
        calcular_temperatura(i);
      }
    }
  }
  HAL_ADC_Stop_DMA(&hadc1);
  
}
/*Estas funciones se utilizan para calcular el valor de 
de cada dato,
Temperatura ºC
Viento Km/h
Sonido Db
Lumoinosidad Lux */

void calcular_temperatura(int i){
   float R=1023.0/adc[i]-1.0;
      
    temperature=1/(log(R)/4275+1/298.15)-273.15;
}
void calcular_luminosidad(int i){
  brightness = exp((float)adc[i]/54.46052);
}
void calcular_sonido(int i){
   if (adc[i]!=0 || adc[i]<0){
        sound= 20*log(adc[i]/100.0)+50;
  }
}
void calcular_viento(int i){
    htim2.Instance->CCR1 = 75 + (adc[i]*1425/MAX_ANGLE);
    speed = (valortotal - 75) * 0.1405;
  
}
/*Esta Función se utiliza para saber cual es nuestro siguiente estado */
int next_state(){
  
  if  (temperature > THRESHOLD_TEMPERATURE && sound <  THRESHOLD_NOISE && speed < THRESHOLD_WIND){
    return 1;
  }
  else if ((temperature < THRESHOLD_TEMPERATURE) && (sound >  THRESHOLD_NOISE) && (speed < THRESHOLD_WIND)){
   return 2;
  }
  else if (temperature < THRESHOLD_TEMPERATURE && sound < THRESHOLD_NOISE && speed> THRESHOLD_WIND){
     return 3;
  }
  else if  (temperature > THRESHOLD_TEMPERATURE && sound >  THRESHOLD_NOISE && speed < THRESHOLD_WIND){
    return 4;
  }
  else if(temperature > THRESHOLD_TEMPERATURE && sound <  THRESHOLD_NOISE && speed > THRESHOLD_WIND){
  return 5 ;
  }
  else if ((temperature < THRESHOLD_TEMPERATURE )&& (sound >  THRESHOLD_NOISE) && (speed > THRESHOLD_WIND)){
    return 6; 
  } else if((temperature > THRESHOLD_TEMPERATURE )&& (sound >  THRESHOLD_NOISE) && (speed > THRESHOLD_WIND)){
      return 7 ;
  }
    
  
}
/*Esta función se utiliza para encender el led según el estado en el que estemos*/

void encederLeds(int caso)
{
  /* PB4 PA7 PA8 VERDES */
  /* PB5 PB6 PB10 ROJOS */

  switch (caso)
  {
  case 0:
    //reset
    //printf("case %i\n", caso);

    GPIOB->ODR |= GPIO_ODR_OD4_Msk;
    GPIOA->ODR |= GPIO_ODR_OD7_Msk;
    GPIOA->ODR |= GPIO_ODR_OD8_Msk;

    GPIOB->ODR &= ~GPIO_ODR_OD5_Msk;
    GPIOB->ODR &= ~GPIO_ODR_OD6_Msk;
    GPIOB->ODR &= ~GPIO_ODR_OD10_Msk;
    break;

  case 1:
    //temperatura se pasa
    //printf("case %i\n", caso);
    GPIOB->ODR |= GPIO_ODR_OD5_Msk;
    GPIOB->ODR &= ~GPIO_ODR_OD4_Msk;
    break;
  case 2:
    //sound se pasa
    //printf("case %i\n", caso);
    //printf("el sound en el case es %d\n",sound);

    GPIOB->ODR |= GPIO_ODR_OD10_Msk;
    GPIOA->ODR &= ~GPIO_ODR_OD8_Msk;
    break;
  case 3:
    //Viento se pasa
   // printf("case %i\n", caso);

    GPIOB->ODR |= GPIO_ODR_OD6_Msk;
    GPIOA->ODR &= ~GPIO_ODR_OD7_Msk;
    break;
  case 4:
    //temperatura se pasa sound se pasa
    //printf("case %i\n", caso);
    GPIOB->ODR |= GPIO_ODR_OD5_Msk;
    GPIOB->ODR &= ~GPIO_ODR_OD4_Msk;
    GPIOB->ODR |= GPIO_ODR_OD10_Msk;
    GPIOA->ODR &= ~GPIO_ODR_OD8_Msk;
    break;

  case 5:
    //Temperatura se pasa viento se pasa
    //printf("case %i\n", caso);
    GPIOB->ODR |= GPIO_ODR_OD5_Msk;
    GPIOB->ODR &= ~GPIO_ODR_OD4_Msk;
    GPIOB->ODR |= GPIO_ODR_OD6_Msk;
    GPIOA->ODR &= ~GPIO_ODR_OD7_Msk;

    break;
  case 6:
    //sound se pasa viento se pasa
    //printf("case %i\n", caso);

    GPIOB->ODR |= GPIO_ODR_OD6_Msk;
    GPIOB->ODR |= GPIO_ODR_OD10_Msk;
    GPIOA->ODR &= ~GPIO_ODR_OD7_Msk;
    GPIOA->ODR &= ~GPIO_ODR_OD8_Msk;
    break;
  case 7:
    //se pasan todos
    //printf("case %i\n", caso);
    GPIOB->ODR |= GPIO_ODR_OD5_Msk;
    GPIOB->ODR &= ~GPIO_ODR_OD4_Msk;
    GPIOB->ODR |= GPIO_ODR_OD10_Msk;
    GPIOA->ODR &= ~GPIO_ODR_OD8_Msk;
    GPIOB->ODR |= GPIO_ODR_OD6_Msk;
    GPIOA->ODR &= ~GPIO_ODR_OD7_Msk;

    break;
  }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
 
    if (onda == 1)
    {
      val1 = __HAL_TIM_GetCounter(htim);
    
      if (val1 < val2)
        valortotal = val2-val1;
      else
      {
        valortotal = val1 - val2;
        HAL_TIM_IC_Stop(&htim3, TIM_CHANNEL_1);
        //val1=__HAL_TIM_GetCounter(htim);
      }
      onda = 0;
    }
    else
    {
      val2 = __HAL_TIM_GetCounter(htim);
      onda = 1;
    }
}

int __io_putchar(int ch)
{
  uint8_t c[1];
  c[0] = ch & 0x00FF;
  HAL_UART_Transmit(&huart2, &*c, 1, 100);
  return ch;
}

int _write(int file,char *ptr, int len)
{
  int DataIdx;
  for(DataIdx= 0; DataIdx< len; DataIdx++)
  {
    __io_putchar(*ptr++);
  }
  return len;
}