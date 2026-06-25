/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define VSTEP 					3.3/4096
#define TX_BUFFER_SIZE 			64
#define RX_BUFFER_SIZE          64

#define CMD_SETPOINT_LONG       "SETPOINT:"
#define CMD_SETPOINT_SHORT      "SET_SP:"

#define CMD_KP_LONG             "KP:"
#define CMD_KP_SHORT            "SET_KP:"

#define CMD_KI_LONG             "KI:"
#define CMD_KI_SHORT            "SET_KI:"

#define CMD_KD_LONG             "KD:"
#define CMD_KD_SHORT            "SET_KD:"

#define CMD_START               "START"
#define CMD_STOP                "STOP"

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

PID_Controller_t pid;
volatile uint32_t adcValue = 0;
float voltage = 0;
char uartTXBuffer[TX_BUFFER_SIZE];

uint8_t rxChar;
char messageBuffer[RX_BUFFER_SIZE];
uint8_t bufferIndex = 0;

volatile uint8_t pidEnabled = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
/* USER CODE BEGIN PFP */
int _write(int file, char *ptr, int len);
float readVoltage(ADC_HandleTypeDef *hadc, volatile uint32_t *adcValue);
uint16_t mapPercentageToDAC(float percent);
void ProcessUARTCommand(char *command);
void ClearUARTBuffer(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len) {
	int i = 0;
	for (i = 0; i < len; i++) {
		ITM_SendChar((*ptr++));
	}
	return len;
}

float readVoltage(ADC_HandleTypeDef *hadc, volatile uint32_t *adcValue) {
	float voltage;

	HAL_ADC_Start(hadc);
	if (HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY) == HAL_OK) {
		*adcValue = HAL_ADC_GetValue(hadc);
	}
	HAL_ADC_Stop(hadc);

	voltage = (float) (*adcValue * VSTEP);

	return voltage;
}

uint16_t mapPercentageToDAC(float percent) {
	if (percent > 100.0f) {
		percent = 100.0f;
	}

	if (percent < 0.0f) {
		percent = 0.0f;
	}

	return (uint16_t) ((percent / 100.0f) * 4095.0f + 0.5f);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_USART2_UART_Init();
	MX_ADC1_Init();
	MX_DAC_Init();
	/* USER CODE BEGIN 2 */
	PID_Init(&pid, 20.0f, 15.0f, 0.2f, 0.1f, 0.0f, 100.0f);
	PID_SetPoint(&pid, 2.5f);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	uint32_t lastTime = HAL_GetTick();
	HAL_UART_Receive_IT(&huart2, &rxChar, 1);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		uint32_t currentTime = HAL_GetTick();
		if (currentTime - lastTime >= 100) {
			voltage = readVoltage(&hadc1, &adcValue);
			if (pidEnabled) {
				pid.output = PID_Compute(&pid, voltage);
			} else {
				pid.output = 0.0f;
			}

			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
					mapPercentageToDAC(pid.output));

			printf("SetPoint: %.3f Voltage: %.4f PIDOutput: %.3f\r\n",
					pid.setPoint, voltage, pid.output);
			snprintf(uartTXBuffer, sizeof(uartTXBuffer),
					"SetPoint: %.3f Voltage: %.4f PIDOutput: %.3f\r\n",
					pid.setPoint, voltage, pid.output);

			HAL_UART_Transmit(&huart2, (uint8_t*) uartTXBuffer,
					strlen(uartTXBuffer), HAL_MAX_DELAY);

			lastTime = currentTime;
		}
		HAL_Delay(5);

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief DAC Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC_Init(void) {

	/* USER CODE BEGIN DAC_Init 0 */

	/* USER CODE END DAC_Init 0 */

	DAC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN DAC_Init 1 */

	/* USER CODE END DAC_Init 1 */

	/** DAC Initialization
	 */
	hdac.Instance = DAC;
	if (HAL_DAC_Init(&hdac) != HAL_OK) {
		Error_Handler();
	}

	/** DAC channel OUT1 config
	 */
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN DAC_Init 2 */

	/* USER CODE END DAC_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : button_Pin */
	GPIO_InitStruct.Pin = button_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(button_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : led_Pin */
	GPIO_InitStruct.Pin = led_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(led_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void ClearUARTBuffer(void) {
	bufferIndex = 0;
	memset(messageBuffer, 0, RX_BUFFER_SIZE);
}

void ProcessUARTCommand(char *command) {
	if (strncmp(command, CMD_SETPOINT_LONG, strlen(CMD_SETPOINT_LONG)) == 0) {
		float userSetPoint = atof(command + strlen(CMD_SETPOINT_LONG));
		PID_SetPoint(&pid, userSetPoint);
	} else if (strncmp(command, CMD_SETPOINT_SHORT, strlen(CMD_SETPOINT_SHORT))
			== 0) {
		float userSetPoint = atof(command + strlen(CMD_SETPOINT_SHORT));
		PID_SetPoint(&pid, userSetPoint);
	} else if (strncmp(command, CMD_KP_LONG, strlen(CMD_KP_LONG)) == 0) {
		float userKp = atof(command + strlen(CMD_KP_LONG));
		PID_SetKp(&pid, userKp);
	} else if (strncmp(command, CMD_KP_SHORT, strlen(CMD_KP_SHORT)) == 0) {
		float userKp = atof(command + strlen(CMD_KP_SHORT));
		PID_SetKp(&pid, userKp);
	} else if (strncmp(command, CMD_KI_LONG, strlen(CMD_KI_LONG)) == 0) {
		float userKi = atof(command + strlen(CMD_KI_LONG));
		PID_SetKi(&pid, userKi);
	} else if (strncmp(command, CMD_KI_SHORT, strlen(CMD_KI_SHORT)) == 0) {
		float userKi = atof(command + strlen(CMD_KI_SHORT));
		PID_SetKi(&pid, userKi);
	} else if (strncmp(command, CMD_KD_LONG, strlen(CMD_KD_LONG)) == 0) {
		float userKd = atof(command + strlen(CMD_KD_LONG));
		PID_SetKd(&pid, userKd);
	} else if (strncmp(command, CMD_KD_SHORT, strlen(CMD_KD_SHORT)) == 0) {
		float userKd = atof(command + strlen(CMD_KD_SHORT));
		PID_SetKd(&pid, userKd);
	} else if (strcmp(command, CMD_START) == 0) {
		pidEnabled = 1;
	} else if (strcmp(command, CMD_STOP) == 0) {
		pidEnabled = 0;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {

		if (rxChar == '\r' || rxChar == '\n') {
			if (bufferIndex > 0) {
				messageBuffer[bufferIndex] = '\0';
				ProcessUARTCommand(messageBuffer);
			}

			ClearUARTBuffer();
		} else {
			if (bufferIndex < RX_BUFFER_SIZE - 1) {
				messageBuffer[bufferIndex++] = (char) rxChar;
				messageBuffer[bufferIndex] = '\0';
			} else {
				ClearUARTBuffer();
			}
		}

		HAL_UART_Receive_IT(&huart2, &rxChar, 1);
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
