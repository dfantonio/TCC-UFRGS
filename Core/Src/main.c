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
#include "gpio.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "signal_processing.h"
#include "stdio.h"
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// TODO: Deve ser 4000
#define ADC_BUF_LEN 4000
// #define ADC_BUF_LEN 100

// Estrutura para armazenar as leituras dos dois canais
typedef struct {
  uint16_t corrente;  // Canal 0 - Leitura de corrente
  uint16_t tensao;    // Canal 1 - Leitura de tensão
} ADC_Leituras_t;

// Declaração externa do array de dados mockados
extern const ADC_Leituras_t mock_adc_data[4000];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
ADC_Leituras_t adc_buf[ADC_BUF_LEN];

arm_rfft_fast_instance_f32 fft_instance;

float32_t fft_result[FFT_LENGTH] = {0};
float32_t fft_temp[FFT_LENGTH * 2];
float32_t fft_mag[FFT_LENGTH / 2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void getPowerMetrics(_Bool firstHalf);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // Inicializa o buffer com dados mockados
  // memcpy(adc_buf, mock_adc_data, sizeof(adc_buf));

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  // volatile uint32_t start_cycles = DWT->CYCCNT;
  // volatile uint32_t elapsed_cycles = DWT->CYCCNT - start_cycles;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  // TODO: Mover a inicialização do DMA para depois da configuração do
  // ADC
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim2);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, ADC_BUF_LEN * 2);

  arm_rfft_fast_init_f32(&fft_instance, FFT_LENGTH);

  // getPowerMetrics(true);
  // getPowerMetrics(false);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
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
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
  getPowerMetrics(true);
}

// Called when buffer is completely filled
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  getPowerMetrics(false);
}

void getPowerMetrics(_Bool firstHalf) {
  int maxLength = ADC_BUF_LEN / 2;

  // Converte para float
  float32_t voltage_buffer[FFT_LENGTH] = {0};
  float32_t current_buffer[FFT_LENGTH] = {0};
  for (int i = 0; i < FFT_LENGTH; i++) {
    if (firstHalf) {
      if (i >= maxLength) {
        voltage_buffer[i] = 0;
        current_buffer[i] = 0;
      } else {
        voltage_buffer[i] = convert_adc_to_float(adc_buf[i].tensao);
        current_buffer[i] = convert_adc_to_float(adc_buf[i].corrente);
      }
    } else {
      if (i >= maxLength) {
        voltage_buffer[i] = 0;
        current_buffer[i] = 0;
      } else {
        voltage_buffer[i] = convert_adc_to_float(adc_buf[i].tensao);
        current_buffer[i] = convert_adc_to_float(adc_buf[i].corrente);
      }
    }
  }

  remove_offset(voltage_buffer, maxLength);
  remove_offset(current_buffer, maxLength);

  // Calcula parâmetros de qualidade
  Quality_Results_t quality =
      calculate_quality_parameters(voltage_buffer, current_buffer, &fft_instance, maxLength);

  // Calcula potências
  Power_Results_t power = calculate_power(voltage_buffer, current_buffer, maxLength);

  char tx_buff[200] = {0};
  // Formata mensagem
  if (!firstHalf) {
    sprintf(tx_buff,
            "V_rms: %.2f V; I_rms: %.2fA; Freq: %.2fHz; "
            "P_act: %.2fW; P_react: %.2fVAR; P_app: %.2fVA; "
            "PF: %.2f\r\n",
            quality.rms_voltage, quality.rms_current, quality.frequency, power.active_power,
            power.reactive_power, power.apparent_power, power.power_factor);

    // sprintf(tx_buff, "V_rms: %.2f V; I_rms: %.2fA; Freq: %.2fHz;\r\n ", quality.rms_voltage,
    //         quality.rms_current, quality.frequency);

    // Envia pela UART
    HAL_UART_Transmit(&huart2, (uint8_t *)tx_buff, strlen(tx_buff), 1000);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  }

  printf("RMS Voltage: %f\r\n", quality.rms_voltage);
  printf("RMS Current: %f\r\n", quality.rms_current);
  printf("Frequency: %f\r\n", quality.frequency);
  printf("Active Power: %f\r\n", power.active_power);
  printf("Reactive Power: %f\r\n", power.reactive_power);
  printf("Apparent Power: %f\r\n", power.apparent_power);
  printf("Power Factor: %f\r\n", power.power_factor);
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
