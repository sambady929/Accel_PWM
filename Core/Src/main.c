#include "main.h"
#include <stdio.h>
#include <string.h>

SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM_Init(void);

void uint82string(uint8_t byte_in, char *string_out);
void uint162string(uint16_t byte_in, char *string_out);
void int82string(int8_t byte_in, char *string_out);
void int162string(int16_t byte_in, char *string_out);
char dig2char(int digit);

void Print_XYZ(int16_t x, int16_t y, int16_t z);
void Accel_Init();
void Accel_Offset_Cal();
void Accel_Get(int16_t *x, int16_t *y, int16_t *z);
void LIS_Read(uint8_t reg, uint8_t *output);
void LIS_Write(uint8_t reg, uint8_t data);

int timeout = 5000;

int main(void)
{

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM_Init();

  int16_t x[100] = {0};
  int16_t y[100] = {0};
  int16_t z[100] = {0};

  TIM_OC_InitTypeDef PWM_OC_Config = {0};

  Accel_Init();

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  PWM_OC_Config.OCMode = TIM_OCMODE_PWM1;
  PWM_OC_Config.Pulse = 500;
  PWM_OC_Config.OCPolarity = TIM_OCPOLARITY_HIGH;
  PWM_OC_Config.OCFastMode = TIM_OCFAST_DISABLE;

  int y_neg = 0;

  while(1){

	  Accel_Get(&x[0], &y[0], &z[0]);
	  HAL_Delay(500);

	  Print_XYZ(x[0], y[0], z[0]);

	  y_neg = y[0]/16;
	  //PWM_OC_Config.Pulse = y_neg;
	  //HAL_TIM_PWM_ConfigChannel(&htim1, &PWM_OC_Config, TIM_CHANNEL_1);

	  /*if (HAL_GPIO_ReadPin(GPIOA, B1_Pin) == GPIO_PIN_SET) {
		  if (dbounceCtr > 10000) {
			  dbounceCtr = 0;
		  }
		  dbounceCtr++;
	  }*/
  }

}

void Print_XYZ(int16_t x, int16_t y, int16_t z) {

	char printout[100];

	char x_accel[]="000000";
	char y_accel[]="000000";
	char z_accel[]="000000";

	int162string(x, x_accel);
	int162string(y, y_accel);
	int162string(z, z_accel);

	strcat(printout, "\r\nData: X=");
	strcat(printout, x_accel);
	strcat(printout, " Y=");
	strcat(printout, y_accel);
	strcat(printout, " Z=");
	strcat(printout, z_accel);

	HAL_UART_Transmit(&huart1, printout, strlen(printout), timeout);


	return;
}

void Accel_Init() {

	uint8_t id_reg = (WHO_AM_I);
	uint8_t response = 0;

	uint8_t ctrl4_w = (CTRL_REG4);
	uint8_t ctrl4_set = (0x77); //400Hz sample

	uint8_t ctrl6_w = (CTRL_REG6);
	uint8_t ctrl6_set = (0x50);

	LIS_Read(id_reg, &response);
	LIS_Read(id_reg, &response);

	LIS_Write(ctrl4_w, ctrl4_set);
	LIS_Write(ctrl6_w, ctrl6_set);

	Accel_Offset_Cal();

	return;

}

void Accel_Offset_Cal() {

	int16_t x_off[] = {0};
	int16_t y_off[] = {0};
	int16_t z_off[] = {0};

	uint8_t x_off_reg = (OFF_X);
	uint8_t y_off_reg = (OFF_Y);
	uint8_t z_off_reg = (OFF_Z);

	int16_t x_off_sum=0;
	int16_t y_off_sum=0;
	int16_t z_off_sum=0;

	//zero the offset calibration
	int8_t clear = (0x00);
	LIS_Write(x_off_reg, clear);
	LIS_Write(y_off_reg, clear);
	LIS_Write(z_off_reg, clear);

	//take average of 10 readings
	int i=0;
	while (i < 10) {
		Accel_Get(&x_off[0], &y_off[0], &z_off[0]);
		x_off_sum += x_off[0]/10;
		y_off_sum += y_off[0]/10;
		z_off_sum += z_off[0]/10;
		i++;
	}

	//Offset register gets multiplied by 32 before being subtracted from accel ADC value
	//Z axis starts off at 1g, which is 16383
	int8_t x_cal = x_off_sum/32;
	int8_t y_cal = y_off_sum/32;
	int8_t z_cal = (z_off_sum-16383)/32;

	LIS_Write(x_off_reg, x_cal);
	LIS_Write(y_off_reg, y_cal);
	LIS_Write(z_off_reg, z_cal);

	return;

}

void Accel_Get(int16_t *x, int16_t *y, int16_t *z) {

	uint8_t output[] = {0, 0, 0, 0, 0, 0};

	uint8_t x_low = (OUT_X_LOW);
	uint8_t x_high = (OUT_X_HIGH);
	uint8_t y_low = (OUT_Y_LOW);
	uint8_t y_high = (OUT_Y_HIGH);
	uint8_t z_low = (OUT_Z_LOW);
	uint8_t z_high = (OUT_Z_HIGH);

	LIS_Read(x_low, &output[0]);
	LIS_Read(x_high, &output[1]);
	LIS_Read(y_low, &output[2]);
	LIS_Read(y_high, &output[3]);
	LIS_Read(z_low, &output[4]);
	LIS_Read(z_high, &output[5]);

	x[0] = (int16_t) (output[1] << 8) | (output[0]);
	y[0] = (int16_t) (output[3] << 8) | (output[2]);
	z[0] = (int16_t) (output[5] << 8) | (output[4]);

	return;

}

void LIS_Read(uint8_t reg, uint8_t *output) {

	uint8_t dummy = (0x00);
	uint8_t read_reg = (0x80) | (reg);

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, &read_reg, &output[0], 1, 5000);
	HAL_SPI_TransmitReceive(&hspi1, &dummy, &output[0], 1, 5000);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

	return;

}

void LIS_Write(uint8_t reg, uint8_t data) {

	uint8_t output[] = {0};

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, &reg, &output[0], 1, 5000);
	HAL_SPI_TransmitReceive(&hspi1, &data, &output[0], 1, 5000);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

	return;

}

void uint82string(uint8_t byte_in, char *string_out) {
	int hundreds=0;
	int tens=0;
	int ones=0;

	int double_digit = byte_in % 100; //returns the last two digits of byte_in
	int single_digit = byte_in % 10; //returns last digit

	hundreds = (byte_in - double_digit)/100;
	tens = (double_digit - single_digit)/10;
	ones = single_digit;

	string_out[0] = dig2char(hundreds);
	string_out[1] = dig2char(tens);
	string_out[2] = dig2char(ones);

	return;

}

void uint162string(uint16_t byte_in, char *string_out) {
	int tenthousands=0;
	int thousands=0;
	int hundreds=0;
	int tens=0;
	int ones=0;

	int quad_digit = byte_in % 10000; //returns the last four digits
	int triple_digit = byte_in % 1000; //returns the last three digits
	int double_digit = byte_in % 100; //returns the last two digits
	int single_digit = byte_in % 10; //returns last digit

	tenthousands = (byte_in - quad_digit)/10000;
	thousands = (quad_digit - triple_digit)/1000;
	hundreds = (triple_digit - double_digit)/100;
	tens = (double_digit - single_digit)/10;
	ones = single_digit;

	string_out[0] = dig2char(tenthousands);
	string_out[1] = dig2char(thousands);
	string_out[2] = dig2char(hundreds);
	string_out[3] = dig2char(tens);
	string_out[4] = dig2char(ones);

	return;

}

void int82string(int8_t byte_in, char *string_out) {
	int sign = 0;
	char uintchar[] = "000";
	uint8_t abs_int = 0;

	sign = byte_in & (1<<7);

	if (sign) {
		string_out[0] = '-';
		abs_int = (uint8_t) ((~(byte_in)) + 1);
		uint82string(abs_int, uintchar);
	} else {
		string_out[0] = '+';
		abs_int = (uint8_t) byte_in;
		uint82string(abs_int, uintchar);
	}

	string_out[1] = uintchar[0];
	string_out[2] = uintchar[1];
	string_out[3] = uintchar[2];

	return;

}

void int162string(int16_t byte_in, char *string_out) {
	int sign = 0;
	char uintchar[] = "00000";
	uint16_t abs_int = 0;

	sign = byte_in & (1<<15);

	if (sign) {
		string_out[0] = '-';
		abs_int = (uint16_t) ((~(byte_in)) + 1);
		uint162string(abs_int, uintchar);
	} else {
		string_out[0] = '+';
		abs_int = (uint16_t) byte_in;
		uint162string(abs_int, uintchar);
	}

	string_out[1] = uintchar[0];
	string_out[2] = uintchar[1];
	string_out[3] = uintchar[2];
	string_out[4] = uintchar[3];
	string_out[5] = uintchar[4];

	return;

}

char dig2char(int digit) {

	char character='\0';

		if (digit == 0)
			character='0';
		if (digit == 1)
			character='1';
		if (digit == 2)
			character='2';
		if (digit == 3)
			character='3';
		if (digit == 4)
			character='4';
		if (digit == 5)
			character='5';
		if (digit == 6)
			character='6';
		if (digit == 7)
			character='7';
		if (digit == 8)
			character='8';
		if (digit == 9)
			character='9';

	return character;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

static void MX_TIM_Init(void) {

	TIM_ClockConfigTypeDef ClockSourceConfig = {0};
	TIM_MasterConfigTypeDef MasterConfig = {0};
	TIM_OC_InitTypeDef PWM_OC_Config = {0};

	htim1.Instance = TIM3;
	htim1.Init.Prescaler = 5;
	htim1.Init.CounterMode = TIM_COUNTERMODE_DOWN;
	htim1.Init.Period = 1000;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 10;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(&htim1);

	ClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim1, &ClockSourceConfig);

	HAL_TIM_PWM_Init(&htim1);

	MasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	MasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim1, &MasterConfig);

	PWM_OC_Config.OCMode = TIM_OCMODE_PWM1;
	PWM_OC_Config.Pulse = 500;
	PWM_OC_Config.OCPolarity = TIM_OCPOLARITY_HIGH;
	PWM_OC_Config.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim1, &PWM_OC_Config, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&htim1, &PWM_OC_Config, TIM_CHANNEL_2);
	HAL_TIM_PWM_ConfigChannel(&htim1, &PWM_OC_Config, TIM_CHANNEL_3);
	HAL_TIM_PWM_ConfigChannel(&htim1, &PWM_OC_Config, TIM_CHANNEL_4);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
