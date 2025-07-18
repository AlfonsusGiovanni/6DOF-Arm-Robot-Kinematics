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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "ArmRobot_Math.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Data command  typedef
typedef enum{
	NO_CMD,
	CALC_FK,
	FK_RESULT,
	CALC_IK,
	IK_RESULT,
	CONNECT_DEVICES,
}Data_Command;

// Data frame typedef
typedef struct{
	uint8_t header1;
	uint8_t header2;
	
	Data_Command cmd;
	
	uint8_t payload_data[24];
}Data_Frame;

// Kinematics typedef
Kinematics_t kinematics;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define HEADER1 0xFF
#define HEADER2 0XAA

#define JOINT_NUM 6

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//--- Text Buffer Variable ---//
char
connected_text[] = "Device Connected \r\n";

//--- Data Buffer ---//
uint8_t
tx_buff[32],
rx_buff[64];

//--- Timer Variable ---//
unsigned long
msgGet_timer;

//--- DH Parameter ---//
float
joint_d[JOINT_NUM] = {
	191.0f, 0.0f, 0.0f,
	575.0f, 0.0f, 65.0f,
},

joint_a[JOINT_NUM] = {
	23.5f, 650.0f, 0.0f,
	0.0f, 0.0f, 0.0f, 
},

joint_alpha[JOINT_NUM] = {
	-90.0f, 0.0f, 90.0f,
	-90.0f, 90.0f, 0.0f,
};

//--- Kinematics Variable ---//
float
input_angle[JOINT_NUM],
input_pos[JOINT_NUM],
output_angle[JOINT_NUM],
output_pos[JOINT_NUM];

uint8_t payload[24];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

void receive_data(void);
bool send_data(uint8_t *data_buffer, uint16_t len);
void reset_buffer(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

Data_Frame received_data;

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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	
	// Turn LED On
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  
	// Kinematics Init
	DHparam_init(&kinematics, joint_d, joint_a, joint_alpha);
	toolframe_init(&kinematics, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	forward_transform_matrix(&kinematics);
	calculate_all_link(&kinematics);
	
	/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		// Receive data from pc
		receive_data();
		
		// Check connect devices command
		if(received_data.cmd == CONNECT_DEVICES){
			tx_buff[0] = HEADER1;
			tx_buff[1] = HEADER2;
			tx_buff[2] = CONNECT_DEVICES;
			
			memcpy(&tx_buff[3], &connected_text, strlen(connected_text)); 
			
			while(!send_data(tx_buff, sizeof(tx_buff)));
			
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			HAL_Delay(100);
			
			reset_buffer();
		}
		
		// Check calc fk command
		else if(received_data.cmd == CALC_FK){
			for(int i=0; i<JOINT_NUM; i++){
				memcpy(&input_angle[i], &received_data.payload_data[i*4], sizeof(float));
			}
			
			run_forward_kinematic(&kinematics, input_angle);
			
			tx_buff[0] = HEADER1;
			tx_buff[1] = HEADER2;
			tx_buff[2] = FK_RESULT;
			
			float tool_pos[6];
			
			for(int i=0; i<JOINT_NUM; i++){
				if(i<3) tool_pos[i] = kinematics.axis_pos_out[i];
				else tool_pos[i] = kinematics.axis_rot_out[i-3];
				
				memcpy(&payload[i*4], &tool_pos[i], sizeof(float));
			}
			
			for(int i=0; i<sizeof(payload); i++){
				tx_buff[i+3] = payload[i];
			}
			
			while(!send_data(tx_buff, sizeof(tx_buff)));
			
			reset_buffer();
		}
		
		// Check calc ik command
		else if(received_data.cmd == CALC_IK){
			for(int i=0; i<JOINT_NUM; i++){
				memcpy(&input_pos[i], &received_data.payload_data[i*4], sizeof(float));
			}
			
			kinematics.j5_enc_angle = input_angle[4];
			run_inverse_kinematic(&kinematics, input_pos);
			
			tx_buff[0] = HEADER1;
			tx_buff[1] = HEADER2;
			tx_buff[2] = IK_RESULT;
			
			for(int i=0; i<JOINT_NUM; i++){
				memcpy(&payload[i*4], &kinematics.joint_ang_out[i], sizeof(float));
			}
			
			for(int i=0; i<sizeof(payload); i++){
				tx_buff[i+3] = payload[i];
			}
			
			while(!send_data(tx_buff, sizeof(tx_buff)));
			
			reset_buffer();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY1_Pin */
  GPIO_InitStruct.Pin = KEY1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*--- RECEIVE DATA FUNCTION ---*/
void receive_data(void){
	// Process Data Buffer
	for(int i=0; i<sizeof(rx_buff); i++){
		if(rx_buff[i] == HEADER1 && rx_buff[i+1] == HEADER2){
			received_data.header1 = rx_buff[i];
			received_data.header2 = rx_buff[i+1];
			
			received_data.cmd = (Data_Command)rx_buff[i+2];
			
			for(int j=0; j<29; j++){
				received_data.payload_data[j] = rx_buff[j+3];
			}
			break;
		}
		else continue;
	}
	
	// Reset Data Buffer
	for(int i=0; i<sizeof(rx_buff); i++){
		rx_buff[i] = 0x00;
	}
}

/*--- SEND DATA FUNCTION ---*/
bool send_data(uint8_t *data_buffer, uint16_t len){
	if(CDC_Transmit_FS(data_buffer, len) == USBD_OK) return true;
	return false;
}

/*--- RESET BUFFER FUNCTION ---*/
void reset_buffer(void){
	received_data.header1 = 0x00;
	received_data.header2 = 0x00;
	received_data.cmd = NO_CMD;
	
	for(int i=0; i<28; i++){
		received_data.payload_data[i] = 0x00;
	}
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
