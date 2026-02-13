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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	uint16_t data_port_b;
	uint16_t data_port_e;
	uint16_t data_port_f;
	uint16_t data_port_g;
} sample_t;

typedef struct {
	int16_t ns_adc;
	int16_t ew_adc;
	uint32_t ns_t;
	uint32_t ew_t;
	uint32_t counter;
} data_record_t;

typedef struct {
	int16_t max_ns_adc;
	int16_t max_ew_adc;
	uint32_t max_ns_t;
	uint32_t max_ew_t;
	float max_ns_inp;
	float max_ew_inp;
} max_sample_t;

typedef struct {
	int16_t min_ns_adc;
	int16_t min_ew_adc;
	uint32_t min_ns_t;
	uint32_t min_ew_t;
} min_sample_t;

typedef struct {
	//uint8_t ns_rec_type;
	int16_t ns_max_adc;
	int16_t ns_min_adc;
	int16_t ew_max_adc;
	int16_t ew_min_adc;

	uint32_t ns_max_t;
	uint32_t ns_min_t;
	uint32_t ew_max_t;
	uint32_t ew_min_t;
} record_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_READ_MAX					1000
#define ADC_R1         				 	10000.0
#define ADC_R2          				4320.0
#define DELAY_SEC						3

#define SETTING_USE_DATASHEET_PARAM
//#define SETTING_INVERT_EW_CHANNEL

#ifdef SETTING_USE_DATASHEET_PARAM
	#define ADC_VREF_P						2.048
	#define ADC_VREF_N						0.988
	#define ADC_OFFSET						1.52
#else
	#define ADC_VREF_P						2.003
	#define ADC_VREF_N						0.994
	#define ADC_OFFSET						1.55
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
UART_HandleTypeDef *esp_uart;
UART_HandleTypeDef *debug_uart;

sample_t sample[SAMPLE_READ_MAX];
record_t rec;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
void find_max();
void find_min();
void find_max_min();
void find_max_min2();
void read_adc();
void send_data();
void clear_buffer();
void gps_uart_send_cmd(char *cmd);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_sec(int sec){
	int count_500ms = 0;
	do{
		if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK){
			Error_Handler();
		}
		HAL_Delay(500);
		count_500ms++;
	}while(count_500ms < (sec<<1));
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  // -- UART Mapping -- //
  esp_uart = &huart2;
  debug_uart = &huart4;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  // Temp for debug
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;

  // Check last reset cause
  printf("Last reset cause : ");
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)){
	  printf("IWDG timer");
  }else if(__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)){
	  printf("Reset Pin");
  }else{
	  printf("Other reason");
  }
  printf("\n");
  __HAL_RCC_CLEAR_RESET_FLAGS();

  printf("Setup :\n");

  // Empty all data in sample buffer
  clear_buffer();

  printf("Main program started\n");
  HAL_GPIO_WritePin(USER_LED_BLUE_GPIO_Port, USER_LED_BLUE_Pin, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1){
	  uint16_t port_d;

	  if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK){
		  Error_Handler();
	  }

	  port_d = GPIOD->IDR;
	  if(port_d & 0x000A){			// Use 0x0002 for TRIG+, 0x0008 for TRIG- or 0x000A for Both trigger
		  // TRIG +
		  read_adc();
		  find_max_min();
		  send_data();
		  clear_buffer();
		  delay_sec(DELAY_SEC);
	  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Window = 4000;
  hiwdg.Init.Reload = 4000;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, USER_LED_GREEN_Pin|USER_LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : GPIO_USER_SW_Pin */
  GPIO_InitStruct.Pin = GPIO_USER_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIO_USER_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D0A_Pin D1A_Pin D2A_Pin D3A_Pin
                           D4A_Pin D5A_Pin D6A_Pin D7A_Pin
                           D8A_Pin D9A_Pin CT2_Pin CT3_Pin
                           CT4_Pin CT5_Pin CT6_Pin */
  GPIO_InitStruct.Pin = D0A_Pin|D1A_Pin|D2A_Pin|D3A_Pin
                          |D4A_Pin|D5A_Pin|D6A_Pin|D7A_Pin
                          |D8A_Pin|D9A_Pin|CT2_Pin|CT3_Pin
                          |CT4_Pin|CT5_Pin|CT6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : CT0_Pin CT1_Pin CT18_Pin CT19_Pin */
  GPIO_InitStruct.Pin = CT0_Pin|CT1_Pin|CT18_Pin|CT19_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CT7_Pin CT8_Pin CT23_Pin CT22_Pin
                           CT21_Pin CT20_Pin D0B_Pin D1B_Pin
                           D2B_Pin D3B_Pin D4B_Pin D5B_Pin
                           D6B_Pin D7B_Pin D8B_Pin D9B_Pin */
  GPIO_InitStruct.Pin = CT7_Pin|CT8_Pin|CT23_Pin|CT22_Pin
                          |CT21_Pin|CT20_Pin|D0B_Pin|D1B_Pin
                          |D2B_Pin|D3B_Pin|D4B_Pin|D5B_Pin
                          |D6B_Pin|D7B_Pin|D8B_Pin|D9B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : CT9_Pin CT10_Pin CT11_Pin CT12_Pin
                           CT13_Pin CT14_Pin CT15_Pin CT16_Pin
                           CT17_Pin */
  GPIO_InitStruct.Pin = CT9_Pin|CT10_Pin|CT11_Pin|CT12_Pin
                          |CT13_Pin|CT14_Pin|CT15_Pin|CT16_Pin
                          |CT17_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USER_LED_GREEN_Pin USER_LED_BLUE_Pin */
  GPIO_InitStruct.Pin = USER_LED_GREEN_Pin|USER_LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD1 PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
#define TB_MODE							TB_MODE_0
#define MAX_BUF_SIZE					500




data_record_t data[SAMPLE_READ_MAX];
max_sample_t max_sample;
min_sample_t min_sample;



int16_t convert_number(uint16_t data){
	int16_t value;
#if TB_MODE == TB_MODE_0
		if((data & 0x0200) > 0){
			// Positive
			value = data & 0x01ff;
		}else{
			// Negative
			value = -(((~data + 1) & 0x01ff));
		}
#else
		if((data & 0x0200) > 0){
			// Negative
			value = -(~data + 0x0001);
		} else{
			// Positive
			value = data & 0x01ff;
		}
#endif
	return value;
}


inline void read_adc(){
	// Read 300 samples
	int count;
	//uint16_t adc_data_a, adc_data_b;
	//int16_t adc_value_a, adc_value_b;
	//uint32_t counter;
	sample_t *p_sample;


	// temp
	//uint32_t counter_first, counter_last;

	// On LED to indicate adc read
	count = 0;
	p_sample = sample;
	do{
		p_sample->data_port_b = GPIOB->IDR;
		p_sample->data_port_e = GPIOE->IDR;
		p_sample->data_port_f = GPIOF->IDR;
		p_sample->data_port_g = GPIOG->IDR;
		p_sample++;
	}while(++count < SAMPLE_READ_MAX);
}


void send_data(){
	char send_buf[MAX_BUF_SIZE];
	uint8_t chksum = 0xa5;
	//sprintf(send_buf, "$STMFIELD,%d,%lu,%d,%lu*%02X\r\n", max_sample.max_ns_adc,max_sample.max_ns_t, max_sample.max_ew_adc, max_sample.max_ew_t, chksum);
	sprintf(send_buf, "$STMFIELD,%d,%lu,%d,%lu,%d,%lu,%d,%lu*%02X\r\n", rec.ns_max_adc, rec.ns_max_t, rec.ns_min_adc, rec.ns_min_t, rec.ew_max_adc, rec.ew_max_t, rec.ew_min_adc,rec.ew_min_t, chksum);
	printf("\nSend update to ESP32 = %s\n",send_buf);
	HAL_UART_Transmit(esp_uart, (uint8_t*)send_buf, strlen(send_buf), HAL_MAX_DELAY);
}




void find_max(){
	float max_ns_v = 0.0, max_ew_v = 0.0;
	int count;
	uint16_t adc_data_a, adc_data_b;
	int16_t adc_value_a, adc_value_b;
	uint32_t counter = 0;
	sample_t *p_sample;
	uint32_t trigger_t;
	uint16_t temp1, temp2;

	//uint32_t counter_first, counter_last;

	count = 0;
	p_sample = sample;
	max_sample.max_ns_adc = 0;
	max_sample.max_ew_adc = 0;

	do{


		// Read ADC
		adc_data_a = p_sample->data_port_f & 0x03ff;
		adc_data_b = (p_sample->data_port_g >> 6) & 0x03ff;

		// Read Counter
		counter = 0;
		// -- CT23-CT20 (4 bits)
		temp1 = p_sample->data_port_g; 	// CT23-PG2, CT22-PG3, CT21-PG4, CT20-PG5
		//temp2 = ((temp1 & 0x0004) << 1) | ((temp1 & 0x0008) >> 1) | ((temp1 & 0x0010) >> 3) | ((temp1 & 0x0020) >> 5);
		//counter = temp2 << 20;
		counter |= ((temp1 & 0x0004) << 21);
		counter |= ((temp1 & 0x0008) << 19);
		counter |= ((temp1 & 0x0010) << 17);
		counter |= ((temp1 & 0x0020) << 15);

		// -- CT19-CT18 (2 bits)
		temp1 = p_sample->data_port_b;	// CT19-PB11, CT18-PB10
		temp2 = (temp1 >> 10) & 0x0003;
		//temp2 = 0xffff;
		counter |= (temp2 << 18);

		// -- CT17-CT9 (9 bits)
		temp1 = p_sample->data_port_e;	// CT17-PE15 ... CT9-PB7
		temp2 = (temp1 >> 7) & 0x01ff;
		counter |= (temp2 << 9);

		// -- CT8-CT7 (2 bits)
		temp1 = p_sample->data_port_g;	// CT8-PG1, CT7-PG0
		temp2 = temp1 & 0x0003;
		counter |= (temp2 << 7);

		// -- CT6-CT2 (5 bits)
		temp1 = p_sample->data_port_f;	// CT6-PF15 ... CT2-PF11
		temp2 = (temp1 >> 11) & 0x001f;
		counter |= (temp2 << 2);

		// -- CT1-CT0 (2 bits)
		temp1 = p_sample->data_port_b;	// CT1-PB2, CT0-PB1
		temp2 = (temp1 >> 1) & 0x0003;
		counter |= temp2;

		// Convert the adc_data to adc_value
		adc_value_a = convert_number(adc_data_a);
		adc_value_b = -(convert_number(adc_data_b));			// *Invert to negative value due to the hardware bug

		// Check if current max is max
		if(adc_value_a > max_sample.max_ns_adc){
			// Store new max adc_value and calculate adc_voltage
			max_sample.max_ns_adc = adc_value_a;
			max_sample.max_ns_t = counter;
			max_ns_v = ((ADC_VREF_P - ADC_VREF_N) * adc_value_a / 512) + ADC_OFFSET;
		}

		if(adc_value_b > max_sample.max_ew_adc){
			// Store new max adc_value and calculate adc_voltage
			max_sample.max_ew_adc = adc_value_b;
			max_sample.max_ew_t = counter;
			max_ew_v = ((ADC_VREF_P - ADC_VREF_N) * adc_value_b / 512) + ADC_OFFSET;
		}

		p_sample++;
		if (count == 0) trigger_t = counter;
	} while(++count < SAMPLE_READ_MAX);

	max_sample.max_ns_inp = (max_ns_v - 1.0) / (ADC_R2 / (ADC_R1 + ADC_R2));
	max_sample.max_ew_inp = (max_ew_v - 1.0) / (ADC_R2 / (ADC_R1 + ADC_R2));

	printf("\033[7A\r");
	printf("----------------------------------------------------------------------------------------------\n");
	printf("\tTrigger+\tBase counter = %lu\n",trigger_t);
	printf("\tMax ADC\t\tMax Voltage\tInput\t\tCounter\t\tTime to Max\n");
	printf("----------------------------------------------------------------------------------------------\n");
	printf("N-S\t%d\t\t%2.4f\t\t%2.4f\t\t%lu\t\t%1.1f\n",max_sample.max_ns_adc, max_ns_v, max_sample.max_ns_inp, max_sample.max_ns_t, (max_sample.max_ns_t-trigger_t)/10.0);
	printf("E-W\t%d\t\t%2.4f\t\t%2.4f\t\t%lu\t\t%1.1f\n",max_sample.max_ew_adc, max_ew_v, max_sample.max_ew_inp, max_sample.max_ew_t, (max_sample.max_ew_t-trigger_t)/10.0);
	printf("----------------------------------------------------------------------------------------------\n");
	printf("\e[7F");
	printf("\x1b[7A\r");

	///printf(" Max ADC = %d, Voltage = %2.4f\n", max_sample.max_ns_adc,max_ns_v);
	//printf(" Trig @ %lu, Max @ %lu, Peak at %d us\n", counter_first, max_sample.max_ns_t, (max_sample.max_ns_t -counter_first) / 10 );
	//printf(" Input = %2.4f V\n", (max_ns_v - 1.0) / (ADC_R2 / (ADC_R1 + ADC_R2)));
	//printf("----------------------------------------------------\n");


	clear_buffer();
}



void find_min(){
	float min_ns_v = 0.0, min_ew_v = 0.0;
	float min_ns_inp = 0.0, min_ew_inp = 0.0;
	int count;
	uint16_t adc_data_a, adc_data_b;
	int16_t adc_value_a, adc_value_b;
	uint32_t counter = 0;
	sample_t *p_sample;

	// temp
	uint32_t trigger_t;

	count = 0;
	p_sample = sample;
	min_sample.min_ns_adc = 0;
	min_sample.min_ew_adc = 0;

	do{
		uint16_t temp1, temp2;

		// Read ADC
		adc_data_a = p_sample->data_port_f & 0x03ff;
		adc_data_b = (p_sample->data_port_g >> 6) & 0x03ff;

		// Read Counter
		counter = 0;
		// -- CT23-CT20 (4 bits)
		temp1 = p_sample->data_port_g; 	// CT23-PG2, CT22-PG3, CT21-PG4, CT20-PG5
		//temp2 = ((temp1 & 0x0004) << 1) | ((temp1 & 0x0008) >> 1) | ((temp1 & 0x0010) >> 3) | ((temp1 & 0x0020) >> 5);
		//counter = temp2 << 20;
		counter |= ((temp1 & 0x0004) << 21);
		counter |= ((temp1 & 0x0008) << 19);
		counter |= ((temp1 & 0x0010) << 17);
		counter |= ((temp1 & 0x0020) << 15);

		// -- CT19-CT18 (2 bits)
		temp1 = p_sample->data_port_b;	// CT19-PB11, CT18-PB10
		temp2 = (temp1 >> 10) & 0x0003;
		counter |= (temp2 << 18);

		// -- CT17-CT9 (9 bits)
		temp1 = p_sample->data_port_e;	// CT17-PE15 ... CT9-PB7
		temp2 = (temp1 >> 7) & 0x01ff;
		counter |= (temp2 << 9);

		// -- CT8-CT7 (2 bits)
		temp1 = p_sample->data_port_g;	// CT8-PG1, CT7-PG0
		temp2 = temp1 & 0x0003;
		counter |= (temp2 << 7);

		// -- CT6-CT2 (5 bits)
		temp1 = p_sample->data_port_f;	// CT6-PF15 ... CT2-PF11
		temp2 = (temp1 >> 11) & 0x001f;
		counter |= (temp2 << 2);

		// -- CT1-CT0 (2 bits)
		temp1 = p_sample->data_port_b;	// CT1-PB2, CT0-PB1
		temp2 = (temp1 >> 1) & 0x0003;
		counter |= temp2;

		// Convert the adc_data to adc_value
		adc_value_a = convert_number(adc_data_a);
		adc_value_b = convert_number(adc_data_b);

		// Check if current max is max
		if(adc_value_a < min_sample.min_ns_adc){
			// Store new max adc_value and calculate adc_voltage
			min_sample.min_ns_adc = adc_value_a;
			min_sample.min_ns_t = counter;
			//min_ns_v = ADC_OFFSET - ((ADC_VREF_P - ADC_VREF_N) * adc_value_a / 512);
			min_ns_v = ((ADC_VREF_P - ADC_VREF_N) * adc_value_a / 512) + ADC_OFFSET;
		}

		if(adc_value_b < min_sample.min_ew_adc){
			// Store new max adc_value and calculate adc_voltage
			min_sample.min_ew_adc = adc_value_b;
			min_sample.min_ew_t = counter;
			min_ew_v = ADC_OFFSET - ((ADC_VREF_P - ADC_VREF_N) * adc_value_b / 512);
		}

		p_sample++;
		if (count == 0) trigger_t = counter;
	} while(++count < SAMPLE_READ_MAX);


	printf("----------------------------------------------------------------------------------------------\n");
	printf("\tTrigger-\tBase counter = %lu\n",trigger_t);
	printf("\tMax ADC\t\tMax Voltage\tInput\t\tCounter\t\tTime to Min\n");
	printf("----------------------------------------------------------------------------------------------\n");
	printf("N-S\t%d\t\t%2.4f\t\t%2.4f\t\t%lu\t\t%1.1f\n",min_sample.min_ns_adc, min_ns_v, min_ns_inp, min_sample.min_ns_t, (min_sample.min_ns_t-trigger_t)/10.0);
	printf("E-W\t%d\t\t%2.4f\t\t%2.4f\t\t%lu\t\t%1.1f\n",min_sample.min_ew_adc, min_ew_v, min_ew_inp, min_sample.min_ew_t, (min_sample.min_ew_t-trigger_t)/10.0);
	printf("----------------------------------------------------------------------------------------------\n");

	clear_buffer();
/*
	printf("----------------------------------------------------\n");
	printf(" Min ADC = %d, Voltage = %2.4f\n", min_sample.min_ns_adc,min_ns_v);
	printf(" Trig @ %lu, Max @ %lu, Peak at %d us\n", counter_first, min_sample.min_ns_t, (min_sample.min_ns_t -counter_first) / 10 );
	printf(" Input = %2.4f V\n", (min_ns_v - 1.0) / (ADC_R2 / (ADC_R1 + ADC_R2)));
	printf("----------------------------------------------------\n");
*/


}



float cal_voltage_adc(int16_t adc_val){
	return ((ADC_VREF_P - ADC_VREF_N) * adc_val / 512) + ADC_OFFSET;
}

float cal_voltage_inp(float vol_adc){
	return ((vol_adc - 1.5) / (ADC_R2 / (ADC_R1 + ADC_R2)));
}

void find_max_min(){

	int count;

	uint16_t adc_data_a, adc_data_b;
	int16_t adc_value_a, adc_value_b;

	uint32_t counter = 0;
	uint32_t trigger_t;
	uint16_t temp1, temp2;

	sample_t *p_sample;

	float ns_max_v, ns_min_v;
	float ew_max_v, ew_min_v;

	float ns_max_inp, ns_min_inp;
	float ew_max_inp, ew_min_inp;

	rec.ns_max_adc = 0;
	rec.ns_min_adc = 0;
	rec.ew_max_adc = 0;
	rec.ew_min_adc = 0;

	count = 0;
	p_sample = sample;
	do{

		// Read ADC
		adc_data_a = p_sample->data_port_f & 0x03ff;
		adc_data_b = (p_sample->data_port_g >> 6) & 0x03ff;

		// Read Counter
		counter = 0;
		// -- CT23-CT20 (4 bits)
		temp1 = p_sample->data_port_g; 	// CT23-PG2, CT22-PG3, CT21-PG4, CT20-PG5
		//temp2 = ((temp1 & 0x0004) << 1) | ((temp1 & 0x0008) >> 1) | ((temp1 & 0x0010) >> 3) | ((temp1 & 0x0020) >> 5);
		//counter = temp2 << 20;
		counter |= ((temp1 & 0x0004) << 21);
		counter |= ((temp1 & 0x0008) << 19);
		counter |= ((temp1 & 0x0010) << 17);
		counter |= ((temp1 & 0x0020) << 15);

		// -- CT19-CT18 (2 bits)
		temp1 = p_sample->data_port_b;	// CT19-PB11, CT18-PB10
		temp2 = (temp1 >> 10) & 0x0003;
		//temp2 = 0xffff;
		counter |= (temp2 << 18);

		// -- CT17-CT9 (9 bits)
		temp1 = p_sample->data_port_e;	// CT17-PE15 ... CT9-PB7
		temp2 = (temp1 >> 7) & 0x01ff;
		counter |= (temp2 << 9);

		// -- CT8-CT7 (2 bits)
		temp1 = p_sample->data_port_g;	// CT8-PG1, CT7-PG0
		temp2 = temp1 & 0x0003;
		counter |= (temp2 << 7);

		// -- CT6-CT2 (5 bits)
		temp1 = p_sample->data_port_f;	// CT6-PF15 ... CT2-PF11
		temp2 = (temp1 >> 11) & 0x001f;
		counter |= (temp2 << 2);

		// -- CT1-CT0 (2 bits)
		temp1 = p_sample->data_port_b;	// CT1-PB2, CT0-PB1
		temp2 = (temp1 >> 1) & 0x0003;
		counter |= temp2;

		// Convert the adc_data to adc_value
		adc_value_a = convert_number(adc_data_a);

#ifdef SETTING_INVERT_EW_CHANNEL
		adc_value_b = -(convert_number(adc_data_b));			// *Invert to negative value due to the hardware bug
#else
		adc_value_b = convert_number(adc_data_b);
#endif

		// Find Max/Min of N-S
		if(adc_value_a > rec.ns_max_adc){
			rec.ns_max_adc = adc_value_a;
			rec.ns_max_t = counter;
		}else if (adc_value_a < rec.ns_min_adc){
			rec.ns_min_adc = adc_value_a;
			rec.ns_min_t = counter;
		}

		// Find Max/Min of E-W
		if(adc_value_b > rec.ew_max_adc){
			rec.ew_max_adc = adc_value_b;
			rec.ew_max_t = counter;
		}else if (adc_value_b < rec.ew_min_adc){
			rec.ew_min_adc = adc_value_b;
			rec.ew_min_t = counter;
		}

		p_sample++;
		if (count == 0) trigger_t = counter;
	} while(++count < SAMPLE_READ_MAX);


	// Calculation
	//ns_max_v = ((ADC_VREF_P - ADC_VREF_N) * rec.ns_max_adc / 512) + ADC_OFFSET;
	//ns_min_v = ((ADC_VREF_P - ADC_VREF_N) * rec.ns_min_adc / 512) + ADC_OFFSET;
	//ew_max_v = ((ADC_VREF_P - ADC_VREF_N) * rec.ew_max_adc / 512) + ADC_OFFSET;
	//ew_min_v = ((ADC_VREF_P - ADC_VREF_N) * rec.ew_min_adc / 512) + ADC_OFFSET;

	ns_max_v = cal_voltage_adc(rec.ns_max_adc);
	ns_min_v = cal_voltage_adc(rec.ns_min_adc);
	ew_max_v = cal_voltage_adc(rec.ew_max_adc);
	ew_min_v = cal_voltage_adc(rec.ew_min_adc);

	ns_max_inp = cal_voltage_inp(ns_max_v);
	ns_min_inp = cal_voltage_inp(ns_min_v);
	ew_max_inp = cal_voltage_inp(ew_max_v);
	ew_min_inp = cal_voltage_inp(ew_min_v);


	// Printing
	printf("\n");
	printf("-------------------------------------------------------------------------------\n");
	printf("\tMax ADC\t\tMax V\t(Inp)\t\tMin ADC\t\tMin V\t(Inp)\n");
	printf("\tMax Cnt\t\tMax us\t\t\tMin Cnt\t\tMin us\n");
	printf("-------------------------------------------------------------------------------\n");
	printf("N-S\t%d\t\t%2.4f\t(%2.4f)\t%d\t\t%2.4f\t(%2.4f)\n", rec.ns_max_adc, ns_max_v, ns_max_inp, rec.ns_min_adc, ns_min_v, ns_min_inp);
	printf("\t%lu\t\t%1.1f\t\t\t%lu\t\t%1.1f\n", rec.ns_max_t, (rec.ns_max_t-trigger_t)/10.0, rec.ns_min_t, (rec.ns_min_t-trigger_t)/10.0);
	printf("\n");
	printf("E-W\t%d\t\t%2.4f\t(%2.4f)\t%d\t\t%2.4f\t(%2.4f)\n", rec.ew_max_adc, ew_max_v, ew_max_inp, rec.ew_min_adc, ew_min_v, ew_min_inp);
	printf("\t%lu\t\t%1.1f\t\t\t%lu\t\t%1.1f\n", rec.ew_max_t, (rec.ew_max_t-trigger_t)/10.0, rec.ew_min_t, (rec.ew_min_t-trigger_t)/10.0);
	printf("\n");
	printf("\tBase counter = %lu\n",trigger_t);



}

/* Send command to GNSS module function
 * the cmd is exclude start symbol ($) and end symbol (*)
 * this function will put '$' and '*XX<CR><LF>' and send over the uart interface
 */
void gps_uart_send_cmd(char *cmd){
	char chksum = 0;
	char *pcmd = cmd;
	do{
		chksum ^= (*pcmd);
		pcmd++;
	}while(*pcmd != '\0');
	printf ("Chksum = 0x%02X\n",chksum);
}

void clear_buffer(){
	sample_t *p_sample;
	int count = 0;

	p_sample = sample;
	count = 0;
	do{
		p_sample->data_port_b = 0;
		p_sample->data_port_e = 0;
		p_sample->data_port_f = 0;
		p_sample->data_port_g = 0;
		p_sample++;
	}while(++count < SAMPLE_READ_MAX);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//if(GPIO_Pin == GPIO_TRIG_P_Pin) {
		//printf("Trigger + detected\n");
		//read_adc();
		//find_max();
		//trigger = 1;
	//	find_max2();
	//} else if (GPIO_Pin == GPIO_TRIG_N_Pin) {
	//	printf("Trigger - detected\n");
	//	read_adc();
	//} else if(GPIO_Pin == GPIO_USER_SW_Pin){
	if(GPIO_Pin == GPIO_USER_SW_Pin){
		printf("User switch pressed\n");
		while(1){
			__NOP();
		}
	} else {
      __NOP();
	}
}


// Overwite the _write function that defined in syscall.c
int _write(int file, char *ptr, int len)
{
    //HAL_UART_Transmit(&huart4, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    HAL_UART_Transmit(debug_uart, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
