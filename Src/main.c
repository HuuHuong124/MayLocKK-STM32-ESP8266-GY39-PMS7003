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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "bme280.h"
#include "i2c-lcd.h"

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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct ValueSensor
{
	uint32_t PM1;		// um/g
	uint32_t PM2_5;	// um/g
	uint32_t PM10_0;	// um/g
	uint32_t Temp;		// 'C
	uint32_t Humi;		// %
	uint32_t Pre;		// pa
	uint32_t FanState;
} ;
struct ValueSensor 		ValueSensor_t;

uint8_t temperature;
uint8_t humidity;
uint16_t pressure;

struct bme280_dev dev;
struct bme280_data comp_data;
int8_t rslt;
 
uint8_t Rx_dataSever[1];
uint8_t Rx_dataDust[2];
uint8_t PmsTransfer_cplt;
uint8_t inFrame = 0;
uint8_t detectOff = 0;
unsigned char Buff_Sensor[63];
uint8_t frameLen;

uint32_t PM1_0_temp;
uint32_t PM2_5_temp;
uint32_t PM10_0_temp;

char line1[16];
char line2[16];

//int nhietdo[16];

uint8_t fan_auto=0;
uint8_t fan=0;
uint8_t get_sever=0;

uint8_t state=0;
uint8_t state_esp=0;
uint8_t state_lcd=0;
uint8_t	send_esp=0;

char LCDprint[30];

void sendDatatoESP();

int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  if(HAL_I2C_Master_Transmit(&hi2c1, (id << 1), &reg_addr, 1, 10) != HAL_OK) return -1;
  if(HAL_I2C_Master_Receive(&hi2c1, (id << 1) | 0x01, data, len, 10) != HAL_OK) return -1;

  return 0;
}

void user_delay_ms(uint32_t period)
{
  HAL_Delay(period);
}

int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  int8_t *buf;
  buf = malloc(len +1);
  buf[0] = reg_addr;
  memcpy(buf +1, data, len);

  if(HAL_I2C_Master_Transmit(&hi2c1, (id << 1), (uint8_t*)buf, len + 1, HAL_MAX_DELAY) != HAL_OK) return -1;

  free(buf);
  return 0;
}
int CheckSumFuncDust(unsigned char *buf)
{
  int checkSum = (buf[30] << 8) + buf[31];
  int sum = 0x42;
  for (int i = 1; i < 30; i++) {
    sum = sum + buf[i];
  }
  return sum == checkSum;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//-Interrupt-Function-for-Read-DUST-Data-------------------------------------------------------------------//
	if (huart->Instance == USART2)	
	{		
		HAL_UART_Receive_IT(&huart2,(uint8_t*)Rx_dataDust, 1);	

		if (inFrame==0) {
			if(Rx_dataDust[0]==0x42 && detectOff==0){
				Buff_Sensor[detectOff]=Rx_dataDust[0];
				detectOff++;
			}
			else if (Rx_dataDust[0]==0x4D && detectOff==1){
				Buff_Sensor[detectOff]=Rx_dataDust[0];
				inFrame = 1;
				detectOff++;
			}
		}
		else {
			Buff_Sensor[detectOff]=Rx_dataDust[0];
			detectOff++;
			frameLen = Buff_Sensor[3]+(Buff_Sensor[2]<<8)+ 4;
			if (detectOff >= frameLen) 
			{
				PmsTransfer_cplt = 1;
			}
		}
		//HAL_UART_Receive_IT(&huart2,(uint8_t*)Rx_dataDust,1);
	}
	if (huart->Instance == USART1)
	{
		HAL_UART_Receive_IT(&huart1,(uint8_t*)Rx_dataSever, 1);
		get_sever=Rx_dataSever[0];
		//printf("get_sever: %d \n",get_sever);
		if(get_sever==50){
			fan =0;
			//printf("sever set tu dong \n");
			fan_auto=1;
		}	
		if(get_sever==49){	
			fan_auto=0;
			//printf("sever set on \n");
			fan=1;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
			ValueSensor_t.FanState=11;
		}	
		if(get_sever==48){		
			fan_auto=0;
			//printf("sever set off\n");
			fan=0;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
			ValueSensor_t.FanState=10;
		}	
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_6){
		//printf("chan PC6 vao ngat\n");
		fan_auto=1-fan_auto;
		if(fan_auto==1) {
			fan =0;
			//printf("chan C6_set tu dong 1\n");
			if(PM2_5_temp>=5) {
				ValueSensor_t.FanState=21;
				sendDatatoESP();
			}
			else {
				ValueSensor_t.FanState=20;
				sendDatatoESP();
			}
		}	
		if(fan_auto==0) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
			ValueSensor_t.FanState=10;
			sendDatatoESP();
			//printf("chan C6_set tu dong 0\n");
		}
	}
	if(GPIO_Pin == GPIO_PIN_7){
		//printf("chan PC7 vao ngat\n");
		//printf("tu dong: %d\n",fan_auto);
		if(fan_auto==0) {
			fan=1-fan;
			if(fan==1) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
				ValueSensor_t.FanState=11;
				sendDatatoESP();
			}
			else {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
				ValueSensor_t.FanState=10;
				sendDatatoESP();
			}
		}
	}	
}
void getPMAS007_Value()
{		
	ValueSensor_t.PM1    = 0;	
	ValueSensor_t.PM2_5  = 0; 
	ValueSensor_t.PM10_0 = 0; 
	if(PmsTransfer_cplt == 1)
		{	
			if(CheckSumFuncDust(Buff_Sensor))
			{
				PM1_0_temp 	= Buff_Sensor[11]+(Buff_Sensor[10]<<8);
				PM2_5_temp	= Buff_Sensor[13]+(Buff_Sensor[12]<<8);
				PM10_0_temp 	= Buff_Sensor[15]+(Buff_Sensor[14]<<8);
				detectOff = 0;
				inFrame = 0;
				PmsTransfer_cplt =0;
				ValueSensor_t.PM1 		=	PM1_0_temp;
				ValueSensor_t.PM2_5		=	PM2_5_temp;
				ValueSensor_t.PM10_0	= PM10_0_temp;	
			}
			//printf("Pm1(ug/m3)     : %d\r\n", ValueSensor_t.PM1);
			//printf("Pm2_5(ug/m3)   : %d\r\n", ValueSensor_t.PM2_5);
			//printf("Pm10_0(ug/m3)  : %d\r\n", ValueSensor_t.PM10_0);
		}
}
	
void GY39_Init()
{
	/* BME280 dat lai */
  dev.dev_id = BME280_I2C_ADDR_PRIM;
  dev.intf = BME280_I2C_INTF;
  dev.read = user_i2c_read;
  dev.write = user_i2c_write;
  dev.delay_ms = user_delay_ms;
  rslt = bme280_init(&dev);
  /* BME280 cai dat */
  dev.settings.osr_h = BME280_OVERSAMPLING_1X;
  dev.settings.osr_p = BME280_OVERSAMPLING_16X;
  dev.settings.osr_t = BME280_OVERSAMPLING_2X;
  dev.settings.filter = BME280_FILTER_COEFF_16;
  rslt = bme280_set_sensor_settings(BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL, &dev);
}
void getGY39_Value()
{		
		ValueSensor_t.Temp   = 0; 
		ValueSensor_t.Humi   = 0;
		ValueSensor_t.Pre    = 0;
		//GY39_Init();
		rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
    dev.delay_ms(40);
    rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
    if(rslt == BME280_OK)
    {
      temperature = comp_data.temperature / 100.0;      /* °C  */
      humidity = comp_data.humidity / 1024.0;           /* %   */
      pressure = comp_data.pressure / 10000.0;          /* hPa */
			ValueSensor_t.Temp 		=	temperature;
			ValueSensor_t.Humi		=	humidity;
			ValueSensor_t.Pre			= pressure;
			//printf("Temp(*C) : %d\n",temperature);
			//printf("Humi(%%)  : %d\n", humidity);
			//printf("Pre(Pa)  : %d\n",pressure);
			//sprintf((char*)nhietdo,"%d",temperature);
		}
}	
void sendDatatoESP()
{
//	//printf("Send Data to ESP: 0");	
	int Buff_ESP[15];	
	Buff_ESP[0]		= 	11;
	Buff_ESP[1]		= 	22;
	Buff_ESP[2] 	= 	ValueSensor_t.Temp; 
	Buff_ESP[3] 	= 	ValueSensor_t.Humi;
	Buff_ESP[4] 	= 	ValueSensor_t.Pre>>8;
	Buff_ESP[5] 	= 	ValueSensor_t.Pre&0xFF; 
	Buff_ESP[6] 	= 	ValueSensor_t.PM1>>8;
	Buff_ESP[7] 	=  	ValueSensor_t.PM1&0xFF; 
	Buff_ESP[8] 	= 	ValueSensor_t.PM2_5>>8; 
	Buff_ESP[9] 	=		ValueSensor_t.PM2_5&0xFF; 
	Buff_ESP[10] 	= 	ValueSensor_t.PM10_0>>8; 
	Buff_ESP[11] 	= 	ValueSensor_t.PM10_0&0xFF; 
	Buff_ESP[12] 	= 	ValueSensor_t.FanState; 
	
	int sumESP = 0;
	
	for(int i=0; i<13; i++)
	{
		sumESP += Buff_ESP[i];
	}	
	Buff_ESP[13] = sumESP>>8; 
	Buff_ESP[14] = sumESP&0xFF; 
	//printf("Buff_ESP_DATA  : ");
	for (int i=0;i<=14;i++)
	{
		HAL_UART_Transmit(&huart1,(uint8_t*)&Buff_ESP[i], 1, 100);	
		//printf("%d\t",Buff_ESP[i]);
	}
	//printf("\r\n");
	////printf("Send Data to ESP: 1");	
}

void LCD_setup()
{
	HAL_Delay(50);
	lcd_init();
	HAL_Delay(50);
	lcd_clear_display();
	lcd_goto_XY (1,1);
	lcd_send_string("MayLocKhongKhi");
	HAL_Delay(1000);
	lcd_clear_display();
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart2,(uint8_t*)Rx_dataDust,1);
	HAL_UART_Receive_IT(&huart1,(uint8_t*)Rx_dataSever,1);

	LCD_setup();
	GY39_Init();
	HAL_Delay(200);
	ValueSensor_t.FanState=10;
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
			if(state==1)
			{
				//printf("----------------Set che do---------------\n");		
				if(fan_auto==1) {				
					if(PM2_5_temp>=5) {
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
						ValueSensor_t.FanState=21;
					}
					else {
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
						ValueSensor_t.FanState=20;
					}
				}
				if(send_esp!=ValueSensor_t.FanState){
					state_esp=58;
					send_esp=ValueSensor_t.FanState;
				}
			}
		if(state==2)
			{
				//printf("-------------------GY-39-----------------\n");
				getGY39_Value();		
			}
		
		if(state==3)
			{
				//printf("------------------PMS7003----------------\n");	
				getPMAS007_Value();
				state =0;
			}
		
		if(state_esp==58)
			{
				//printf("--------------Send data ESP--------------\n");			
				sendDatatoESP();
				state_esp=0;
			}
			
		lcd_clear_display();
		lcd_goto_XY(1,2);
		lcd_send_string("*C|H%| Pa | ug");
		sprintf(LCDprint,"%2d|%2d|%4d|%3d",ValueSensor_t.Temp,ValueSensor_t.Humi,ValueSensor_t.Pre,ValueSensor_t.PM2_5);
		lcd_goto_XY(2,2);
		lcd_send_string(LCDprint);
		sprintf(LCDprint,"%2d",state_esp);
		lcd_goto_XY(1,0);
		lcd_send_string(LCDprint);
		sprintf(LCDprint,"%2d",ValueSensor_t.FanState);
		lcd_goto_XY(2,0);
		lcd_send_string(LCDprint);
			
		state++;
		state_esp++;
		state_lcd++;
		HAL_Delay(1000);		
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
//#ifdef __GNUC__
//  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
//     set to 'Yes') calls __io_putchar() */
//  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//  #define GETCHAR_PROTOTYPE int fgetc(FILE *f)
//#endif /* __GNUC__ */
//	PUTCHAR_PROTOTYPE
//{
//  /* Place your implementation of fputc here */
//  /* e.g. write a character to the USART */
//	HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 100);
//  /* Loop until the end of transmission */
//  return ch;
//}
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
     tex: //printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
