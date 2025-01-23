/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "app_x-cube-ai.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ai_datatypes_defines.h"
#include "ai_platform.h"
#include "network.h"
#include "network_data.h"
#include "board.h"
#include "lcd.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLES_NUMBER	10
#define APP_STRING_SIZE 25
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef struct {
	float ammonia;
	float nitrite;
	float temperature;
	float pH;
}input_data_t;

static input_data_t samples_input[SAMPLES_NUMBER] = {
		{1.325, 0.075, 25.000, 7.850},
		{0.500, 0.450, 27.500, 7.750},
		{0.432, 0.340, 26.500, 7.800},
		{0.285, 0.500, 27.500, 7.650},
		{0.000, 0.290, 26.000, 7.800},
		{0.250, 0.300, 26.000, 7.850},
		{0.000, 0.190, 25.600, 7.900},
		{1.500, 0.125, 25.000, 7.900},
		{0.810, 0.500, 29.000, 7.100},
		{0.100, 0.400, 26.900, 7.500},

};

static float expected_output[SAMPLES_NUMBER] = {
		6.600,
		6.300,
		6.800,
		6.700,
		6.600,
		7.000,
		6.400,
		6.500,
		5.700,
		8.000,

};

float inference_output[SAMPLES_NUMBER] = {0};
uint8_t app_rx_buffer[APP_STRING_SIZE] = {0};
bool demo_mode = true;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void appInitLCD(void);
void appDisplayInference(float inference_value, float expected_value, uint32_t ticks);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Global handle to reference the instantiated C-model */
static ai_handle network = AI_HANDLE_NULL;

/* Global c-array to handle the activations buffer */
AI_ALIGNED(32)
static ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];

/* Array to store the data of the input tensor */
AI_ALIGNED(32)
static ai_float in_data[AI_NETWORK_IN_1_SIZE];
/* or static ai_u8 in_data[AI_NETWORK_IN_1_SIZE_BYTES]; */

/* c-array to store the data of the output tensor */
AI_ALIGNED(32)
static ai_float out_data[AI_NETWORK_OUT_1_SIZE];
/* static ai_u8 out_data[AI_NETWORK_OUT_1_SIZE_BYTES]; */

/* Array of pointer to manage the model's input/output tensors */
static ai_buffer *ai_input;
static ai_buffer *ai_output;

/*
 * Bootstrap
 */
int aiInit(void) {
  ai_error err;

  /* Create and initialize the c-model */
  const ai_handle acts[] = { activations };
  err = ai_network_create_and_init(&network, acts, NULL);
  if (err.type != AI_ERROR_NONE)
  {
	  return 0;
  }
  /* Retrieve pointers to the model's input/output tensors */
  ai_input = ai_network_inputs_get(network, NULL);
  ai_output = ai_network_outputs_get(network, NULL);

  return 0;
}

/*
 * Run inference
 */
int aiRun(const void *in_data, void *out_data) {
  ai_i32 n_batch;
  ai_error err;

  /* 1 - Update IO handlers with the data payload */
  ai_input[0].data = AI_HANDLE_PTR(in_data);
  ai_output[0].data = AI_HANDLE_PTR(out_data);

  /* 2 - Perform the inference */
  n_batch = ai_network_run(network, &ai_input[0], &ai_output[0]);
  if (n_batch != 1) {
      err = ai_network_get_error(network);
  };

  return 0;
}

int aiPreProcess(ai_float* input_array, int index) {

	input_array[0] = samples_input[index].ammonia; // Ammonia value
	input_array[1] = samples_input[index].nitrite; // Nitrite
	input_array[2] = samples_input[index].temperature;	// Temperature
	input_array[3] = samples_input[index].pH;  // pH

	return 0;
}

int aiPostProcess(ai_float* inference_value, int index, uint32_t inference_time) {
	char temp[7] = {0};
	//Store inference output on comparison buffer
	inference_output[index] = inference_value[0];

	appDisplayInference(inference_value[0], expected_output[index], inference_time);
	sprintf(&temp, "%.3f\n", inference_output[index]);
	HAL_UART_Transmit(&huart4, temp, 7, 100);

//	CDC_Transmit_HS(temp, 5);
	return 0;
}

void demoApp(void) {
	uint32_t ticks;

	for(int i = 0; i < SAMPLES_NUMBER; i++) {
		/* Put data onto input buffers */
		aiPreProcess(in_data, i);
		ticks = HAL_GetTick();
		/* Call inference engine */
		aiRun(in_data, out_data); // All the network initialization and in/out handling is done in 'app_x-cube-ai.c'
		/* Post-process data */
		ticks = HAL_GetTick() - ticks;
		aiPostProcess(out_data, i, ticks);
	}
}

void evalApp(void){
	uint8_t tx_buffer[7] = "READY\n";
	uint8_t rx_buffer[APP_STRING_SIZE] = {0};
	uint32_t timeout = 0;
	uint8_t compare_result = 1;
	input_data_t sample_data = {0};
	ai_float input_array[4] = {0};
	ai_float inference_output[1] = {0};

	/* Send the READY response to the application */
	HAL_UART_Transmit(&huart4, tx_buffer, sizeof(tx_buffer), 10);
	/* Receive the first sample */
	HAL_UART_Receive(&huart4, rx_buffer, sizeof(rx_buffer), 10000);
	/* Compare received string with END message*/
	compare_result =(strncmp(rx_buffer, "END", 3));
	while((compare_result) && (rx_buffer[0] != 0)) {
		/* Process samples */
		sscanf((char*)rx_buffer, "%f %f %f %f",
				&sample_data.ammonia,
				&sample_data.nitrite,
				&sample_data.temperature,
				&sample_data.pH);
		input_array[0] = sample_data.ammonia;
		input_array[1] = sample_data.nitrite;
		input_array[2] = sample_data.temperature;
		input_array[3] = sample_data.pH;

		aiRun(input_array, inference_output);
		sprintf(tx_buffer, "%.3f", inference_output[0]);
		HAL_UART_Transmit(&huart4, tx_buffer, sizeof(tx_buffer), 100);
		memset(rx_buffer, 0x00, sizeof(rx_buffer));
		HAL_UART_Receive(&huart4, rx_buffer, sizeof(rx_buffer), 10000);
		compare_result =(strncmp(rx_buffer, "END", 3));
	}
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
/* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_SPI4_Init();
  MX_TIM1_Init();
  MX_UART4_Init();
//  MX_X_CUBE_AI_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t ticks = HAL_GetTick();
  int samples_counter = 0;
  appInitLCD();
  HAL_Delay(1500);
  LCD_SetBrightness(0);
  ST7735_FillRect(&st7735_pObj, 0,0, ST7735Ctx.Width, 80, BLACK);
  uint8_t text[30]= {0};
  sprintf((char *)&text, "Device took %" PRIu32 " ms to boot!", (HAL_GetTick() - ticks));
  LCD_ShowString(4, 4, 160, 16, 16, text);

  aiInit();
  while (1)
  {
    if (demo_mode) {
    	evalApp();
    } else {
      samples_counter = 1;
    }
//	  for(int i = 0; i < SAMPLES_NUMBER; i++) {
//	  /* Put data onto input buffers */
//	  aiPreProcess(in_data, i);
//	  ticks = HAL_GetTick();
//	  /* Call inference engine */
//	  aiRun(in_data, out_data); // All the network initialization and in/out handling is done in 'app_x-cube-ai.c'
//	  /* Post-process data */
//	  ticks = HAL_GetTick() - ticks;
//	  aiPostProcess(out_data, i, ticks);

	  HAL_Delay(1000);
//	  }
	 __asm("NOP");
    /* USER CODE END WHILE */

//  MX_X_CUBE_AI_Process();
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 34;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//	char ref_string[] = "start";
//	if (!(strcmp(app_rx_buffer, ref_string))) {
//		demo_mode = false;
//	}
//	HAL_UART_Receive_IT(&huart4, app_rx_buffer, APP_STRING_SIZE);
//}
void appInitLCD(void){
	uint8_t text[20] = {0};

	ST7735Ctx.Orientation = ST7735_ORIENTATION_LANDSCAPE_ROT180;
	ST7735Ctx.Panel = HannStar_Panel;
	ST7735Ctx.Type = ST7735_0_9_inch_screen;

	ST7735_RegisterBusIO(&st7735_pObj,&st7735_pIO);
	ST7735_LCD_Driver.Init(&st7735_pObj,ST7735_FORMAT_RBG565,&ST7735Ctx);
	ST7735_LCD_Driver.ReadID(&st7735_pObj,&st7735_id);

	LCD_SetBrightness(0);

	ST7735_LCD_Driver.FillRect(&st7735_pObj, 0, 0, ST7735Ctx.Width,ST7735Ctx.Height, BLACK);
	sprintf((char *)&text, "Starting App");
	LCD_ShowString(4, 4, ST7735Ctx.Width, 16, 16, text);
	sprintf((char *)&text, "Booting up device");
	LCD_ShowString(4, 22, ST7735Ctx.Width, 16, 16, text);

	LCD_Light(100, 200);

}

void appDisplayInference(float inference_value, float expected_value, uint32_t ticks) {
	uint8_t string_expected[50] = {0};
	uint8_t string_inference[50] = {0};
	uint8_t string_time[30] = {0};
	uint8_t string_mse[30] = {0};
	float mse_result = 0;

	mse_result = powf((inference_value - expected_value), 2);

	LCD_SetBrightness(0);
	ST7735_LCD_Driver.FillRect(&st7735_pObj, 0, 0, ST7735Ctx.Width,ST7735Ctx.Height, BLACK);

	sprintf((char *)&string_expected, "Expected: %.3f", expected_value);
	LCD_ShowString(4, 4, ST7735Ctx.Width, 16, 16, string_expected);

	sprintf((char *)&string_inference, "Inference: %.3f", inference_value);
	LCD_ShowString(4, 22, ST7735Ctx.Width, 16, 16, string_inference);

	sprintf((char *)&string_mse, "MSE: %.4f", mse_result);
	LCD_ShowString(4, 40, ST7735Ctx.Width, 16, 16, string_mse);

	sprintf((char *)&string_time, "Time: %" PRIu32 " ms", ticks);
	LCD_ShowString(4, 60, ST7735Ctx.Width, 16, 16, string_time);
}
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
