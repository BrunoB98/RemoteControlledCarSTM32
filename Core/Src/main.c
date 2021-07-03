/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

/* Private includes ----------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "stepper.h"
#include "stm32l475e_iot01.h"
#include "stm32l475e_iot01_tsensor.h"
#include "stm32l475e_iot01_hsensor.h"
#include "stm32l475e_iot01_gyro.h"
#include "stm32l475e_iot01_psensor.h"
#include "stm32l475e_iot01_magneto.h"

/* Private typedef -----------------------------------------------------------*/

#define TERMINAL_USE

/* Update SSID and PASSWORD with own Access point settings */
#define SSID     "Redmi"
#define PASSWORD "BBMito98"

uint8_t RemoteIP[] = {192,168,43,234};
#define RemotePORT	8002

#define WIFI_WRITE_TIMEOUT 10000
#define WIFI_READ_TIMEOUT  10000

#define CONNECTION_TRIAL_MAX          10

uint8_t  MAC_Addr[6];
  	    uint8_t  IP_Addr[4];
  	    uint8_t TxData[] = "STM32 : Hello!\n";
  	    int32_t Socket = -1;
  	    uint16_t Datalen;
  	    int32_t ret;
  	    int16_t Trials = CONNECTION_TRIAL_MAX;


#if defined (TERMINAL_USE)
#define TERMOUT(...)  printf(__VA_ARGS__)
#else
#define TERMOUT(...)
#endif

/* Private variables ---------------------------------------------------------*/
#if defined (TERMINAL_USE)
extern UART_HandleTypeDef hDiscoUart;
#endif /* TERMINAL_USE */
static uint8_t RxData [500];


/* Private function prototypes -----------------------------------------------*/
#if defined (TERMINAL_USE)
#ifdef __GNUC__
/* With GCC, small TERMOUT (option LD Linker->Libraries->Small TERMOUT
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
#endif /* TERMINAL_USE */

/* Private function prototypes -----------------------------------------------*/
extern  SPI_HandleTypeDef hspi;


/* Private define ------------------------------------------------------------*/
#define SERVO_INIT 73
#define STEP_DIR_INIT 1
#define STEP_RPM_INIT 8
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;
UART_HandleTypeDef huart1;

osSemaphoreId semaphoreWifiHandle;
osSemaphoreId semaphoreWifiSincHandle;
osSemaphoreId semaphoreSerialHandle;
osSemaphoreId semaphoreServoSincHandle;
osSemaphoreId semaphoreServoHandle;
osSemaphoreId semaphoreStepperHandle;
osSemaphoreId semaphoreMutexHandle;


osThreadId defaultTaskHandle;
osThreadId ServoTaskHandle;
osThreadId StepperTaskHandle;
osThreadId DistanceTaskHandle;
osThreadId ManagerTaskHandle;
osThreadId SerialTaskHandle;
osThreadId SensorsTaskHandle;
osThreadId WifiTaskHandle;


int stepper_rpm;
int stepper_dir;
int16_t pDataXYZ[3] = {0};
float pGyroDataXYZ[3] = {0};
uint8_t command[1];
bool problem = false;
bool connection = false;
bool on = false;

double var;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);

void StartDefaultTask(void const * argument);
void StartServo(void const * argument);
void StartStepper(void const * argument);
void StartDistance(void const * argument);
void StartManager(void const * argument);
void StartSensors(void const * argument);
void StartWifi(void const * argument);


/* Private user code ---------------------------------------------------------*/


/* this function generates ACTIVE delay in microseconds */
void delayStepM (uint16_t us)
{
  __HAL_TIM_SET_COUNTER(&htim8, 0);
  while (__HAL_TIM_GET_COUNTER(&htim8) < us);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	osSemaphoreRelease(semaphoreSerialHandle);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
    case (GPIO_PIN_1):
    {
      SPI_WIFI_ISR();
      break;
    }
    default:
    {
      break;
    }
  }
}

void SPI3_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi);
}

int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 10);
	return len;
}

int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure LED2 */
  BSP_LED_Init(LED2);
  MX_USART1_UART_Init();

  /* Configure the system clock */
   SystemClock_Config();

  /* USER CODE BEGIN Init */
  BSP_GYRO_Init();
  BSP_MAGNETO_Init();
  BSP_ACCELERO_Init();
  BSP_HSENSOR_Init();
  BSP_TSENSOR_Init();
  BSP_PSENSOR_Init();
  /* USER CODE END Init */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Base_Start(&htim8);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  osSemaphoreDef(semaphoreWifiSinc);
  semaphoreWifiSincHandle = osSemaphoreCreate(osSemaphore(semaphoreWifiSinc), 1);

  osSemaphoreDef(semaphoreWifi);
  semaphoreWifiHandle = osSemaphoreCreate(osSemaphore(semaphoreWifi), 1);

  osSemaphoreDef(semaphoreSerial);
  semaphoreSerialHandle = osSemaphoreCreate(osSemaphore(semaphoreSerial), 1);

  osSemaphoreDef(semaphoreServoSinc);
  semaphoreServoSincHandle = osSemaphoreCreate(osSemaphore(semaphoreServoSinc), 1);

  //osSemaphoreDef(semaphoreBuzzer);
  //semaphoreBuzzerHandle = osSemaphoreCreate(osSemaphore(semaphoreBuzzer), 1);

  osSemaphoreDef(semaphoreStepper);
  semaphoreStepperHandle = osSemaphoreCreate(osSemaphore(semaphoreStepper), 1);

  osSemaphoreDef(semaphoreServo);
  semaphoreServoHandle = osSemaphoreCreate(osSemaphore(semaphoreServo), 1);

  osSemaphoreDef(semaphoreMutex);
  semaphoreMutexHandle = osSemaphoreCreate(osSemaphore(semaphoreMutex), 1);


  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ServoTask */
  osThreadDef(ServoTask, StartServo, osPriorityNormal, 0, 128);
  ServoTaskHandle = osThreadCreate(osThread(ServoTask), NULL);

  /* definition and creation of StepperTask */
  osThreadDef(StepperTask, StartStepper, osPriorityNormal, 0, 128);
  StepperTaskHandle = osThreadCreate(osThread(StepperTask), NULL);

  /* definition and creation of DistanceTask */
  osThreadDef(DistanceTask, StartDistance, osPriorityNormal, 0, 128);
  DistanceTaskHandle = osThreadCreate(osThread(DistanceTask), NULL);

  /* definition and creation of ManagerTask */
  osThreadDef(ManagerTask, StartManager, osPriorityAboveNormal, 0, 128);
  ManagerTaskHandle = osThreadCreate(osThread(ManagerTask), NULL);

  /* definition and creation of SensorsTask */
  osThreadDef(SensorsTask, StartSensors, osPriorityNormal, 0, 128);
  SensorsTaskHandle = osThreadCreate(osThread(SensorsTask), NULL);

  /* definition and creation of WifiTask */
   osThreadDef(WifiTask, StartWifi, osPriorityNormal, 0, 128);
   WifiTaskHandle = osThreadCreate(osThread(WifiTask), NULL);


  HAL_UART_Receive_IT(&huart3, command, 1);


  /* Start scheduler */
  osKernelStart();

  /* Infinite loop */
  while (1) {

  }
}



/* System Clock Configuration */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 9;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_UART4;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* TIM2 Initialization Function */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1440-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);
}

/* TIM8 Initialization Function */
static void MX_TIM8_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 72-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65534;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USART1 Initialization Function */
static void MX_USART1_UART_Init(void)
{
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
}

/* USART3 Initialization Function */

static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* GPIO Initialization Function */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STEP_N3_GPIO_Port, STEP_N3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STEP_N4_Pin|STEP_N2_Pin|STEP_N1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : STEP_N3_Pin */
  GPIO_InitStruct.Pin = STEP_N3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STEP_N3_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STEP_N4_Pin STEP_N2_Pin STEP_N1_Pin */
  GPIO_InitStruct.Pin = STEP_N4_Pin|STEP_N2_Pin|STEP_N1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  for(;;){
	  //printf("VAFFANCULO \r\n\n");
  }
}

/* USER CODE BEGIN Header_StartServo */
void StartServo(void const * argument)
{
	htim2.Instance->CCR1 = SERVO_INIT;
	for(;;) {
	  osSemaphoreWait(semaphoreServoSincHandle, osWaitForever);
	  while(htim2.Instance->CCR1 > SERVO_INIT) {
		  htim2.Instance->CCR1 -=1;
		  osDelay(70);
	  }
	  while(htim2.Instance->CCR1 < SERVO_INIT) {
		  htim2.Instance->CCR1 +=1;
		  osDelay(70);
	  }
	}
}

/* USER CODE BEGIN Header_StepperStart */
void StartStepper(void const * argument)
{
  for(;;)
  {
	  osSemaphoreWait(semaphoreMutexHandle, osWaitForever);
	  if((problem == false) && (on == true)){
		  stepper_step_angle(10, stepper_dir, stepper_rpm);
		  osSemaphoreRelease(semaphoreMutexHandle);
		  osDelay(20);
	  } else {
		  osSemaphoreRelease(semaphoreMutexHandle);
		  osSemaphoreWait(semaphoreStepperHandle, osWaitForever);
	  }
  }
}

void StartWifi(void const * argument)
{
	for(;;) {
		osSemaphoreWait(semaphoreWifiSincHandle, osWaitForever);

		#if defined (TERMINAL_USE)
  	    /* Initialize all configured peripherals */
  	    hDiscoUart.Instance = DISCOVERY_COM1;
  	    hDiscoUart.Init.BaudRate = 115200;
  	    hDiscoUart.Init.WordLength = UART_WORDLENGTH_8B;
  	    hDiscoUart.Init.StopBits = UART_STOPBITS_1;
  	    hDiscoUart.Init.Parity = UART_PARITY_NONE;
  	    hDiscoUart.Init.Mode = UART_MODE_TX_RX;
  	    hDiscoUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  	    hDiscoUart.Init.OverSampling = UART_OVERSAMPLING_16;
  	    hDiscoUart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  	    hDiscoUart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  	    BSP_COM_Init(COM1, &hDiscoUart);
  	  #endif /* TERMINAL_USE */

  	    printf("****** WIFI Module in TCP Client mode demonstration ****** \r\n\n");
  	    printf("TCP Client Instructions :\r\n");
  	    printf("1- Make sure your Phone is connected to the same network that\r\n");
  	    printf("   you configured using the Configuration Access Point.\r\n");
  	    printf("2- Create a server by using the android application TCP Server\r\n");
  	    printf("   with port(8002).\r\n");
  	    printf("3- Get the Network Name or IP Address of your Android from the step 2.\r\n\n");



  	    /*Initialize  WIFI module */
  	    if(WIFI_Init() ==  WIFI_STATUS_OK)
  	    {
  	      printf("> WIFI Module Initialized.\r\n");
  	      if(WIFI_GetMAC_Address(MAC_Addr) == WIFI_STATUS_OK)
  	      {
  	        printf("> es-wifi module MAC Address : %X:%X:%X:%X:%X:%X\r\n", MAC_Addr[0], MAC_Addr[1], MAC_Addr[2], MAC_Addr[3], MAC_Addr[4], MAC_Addr[5]);
  	      }
  	      else
  	      {
  	        printf("> ERROR : CANNOT get MAC address\r\n");
  	        BSP_LED_On(LED2);
  	      }

  	      if( WIFI_Connect(SSID, PASSWORD, WIFI_ECN_WPA2_PSK) == WIFI_STATUS_OK)
  	      {
  	        printf("> es-wifi module connected \r\n");
  	        if(WIFI_GetIP_Address(IP_Addr) == WIFI_STATUS_OK)
  	        {
  	          printf("> es-wifi module got IP Address : %d.%d.%d.%d\r\n", IP_Addr[0], IP_Addr[1], IP_Addr[2], IP_Addr[3]);

  	          printf("> Trying to connect to Server: %d.%d.%d.%d:%d ...\r\n", RemoteIP[0], RemoteIP[1], RemoteIP[2],RemoteIP[3], RemotePORT);

  	          while (Trials--)
  	          {
  	            if( WIFI_OpenClientConnection(0, WIFI_TCP_PROTOCOL, "TCP_CLIENT", RemoteIP, RemotePORT, 0) == WIFI_STATUS_OK)
  	            {
  	              printf("> TCP Connection opened successfully.\r\n");
  	              Socket = 0;
  	              connection = true;
  	              osSemaphoreRelease(semaphoreWifiHandle);
  	              break;
  	            }
  	          }
  	          if(Socket == -1)
  	          {
  	            printf("> ERROR : Cannot open Connection\r\n");
  	            BSP_LED_On(LED2);
  	          }
  	        }
  	        else
  	        {
  	          printf("> ERROR : es-wifi module CANNOT get IP address\r\n");
  	          BSP_LED_On(LED2);
  	        }
  	      }
  	      else
  	      {
  	        printf("> ERROR : es-wifi module NOT connected\r\n");
  	        BSP_LED_On(LED2);
  	      }
  	    }
  	    else
  	    {
  	      printf("> ERROR : WIFI Module cannot be initialized.\r\n");
  	      BSP_LED_On(LED2);
  	    }
	}


}

/* USER CODE BEGIN Header_StartDistance */
void StartDistance(void const * argument)
{
  	 for (;;)
	    {
	    	osDelay(3000);
	    	while (connection == false) {
	    		osSemaphoreWait(semaphoreWifiHandle, osWaitForever);
	    	}
	    		//osSemaphoreRelease(semaphoreWifiHandle);


	      if(Socket != -1)
	      {
	        /*ret = WIFI_ReceiveData(Socket, RxData, sizeof(RxData)-1, &Datalen, WIFI_READ_TIMEOUT);
	        if(ret == WIFI_STATUS_OK)
	        {
	          if(Datalen > 0)
	          {
	            RxData[Datalen]=0;
	            //TERMOUT("Received: %s\r\n",RxData);*/
	            ret = WIFI_SendData(Socket, TxData, sizeof(TxData), &Datalen, WIFI_WRITE_TIMEOUT);
	            if (ret != WIFI_STATUS_OK)
	            {
	              //TERMOUT("> ERROR : Failed to Send Data, connection closed\r\n");
	            	connection = false;
	            }
	         // }
	        //}
	        /*else
	        {
	          //TERMOUT("> ERROR : Failed to Receive Data, connection closed\r\n");
	          break;
	        }*/
	      }
	    }
}


/* USER CODE BEGIN Header_StartSensors */
void StartSensors(void const * argument)
{
  for(;;)
  {
	  //BSP_MAGNETO_GetXYZ(pDataXYZ);
	  //printf("MAGNETO: X = %d     Y = %d     Z = %d\r\n\n", pDataXYZ[0], pDataXYZ[1], pDataXYZ[2]);
	  //BSP_GYRO_GetXYZ(pGyroDataXYZ);
	  //printf("GYRO: X = %f     Y = %f     Z = %f\r\n\n", pGyroDataXYZ[0], pGyroDataXYZ[1], pGyroDataXYZ[2]);
	  BSP_ACCELERO_AccGetXYZ(pDataXYZ);
	  osSemaphoreWait(semaphoreMutexHandle, osWaitForever);
	  if ((pDataXYZ[0] < -200) || (pDataXYZ[0] > 200) || (pDataXYZ[3] < 0)) {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		  problem = true;
	  } else {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		  problem = false;
		  osSemaphoreRelease(semaphoreStepperHandle);
	  }
	  osSemaphoreRelease(semaphoreMutexHandle);
	  //printf("ACCELERO: X = %d     Y = %d     Z = %d\r\n\n", pDataXYZ[0], pDataXYZ[1], pDataXYZ[2]); */
	  /*printf("HUMIDITY: %.2f \r\n", BSP_HSENSOR_ReadHumidity());

	  printf("TEMPERATURE: %.2f °C\r\n", BSP_TSENSOR_ReadTemp());*/
	  //BSP_TSENSOR_ReadTemp();
	  //BSP_PSENSOR_ReadPressure();
	  //printf("PRESSURE: %.2f \r\n\n", BSP_PSENSOR_ReadPressure());
	  osDelay(500);
  }
}

/* USER CODE BEGIN Header_StartManager */
void StartManager(void const * argument)
{
  for(;;)
  {
	  //printf("Sono manager\r\n");
	  osSemaphoreWait(semaphoreSerialHandle, osWaitForever);
	  HAL_UART_Receive_IT(&huart3, command, 1);
	  osSemaphoreWait(semaphoreMutexHandle, osWaitForever);
	  switch (command[0]) {
			case (100): //accensione
				if(stepper_rpm == 0) {
					//printf("Accensione \r\n");
					on = true;
					stepper_rpm = STEP_RPM_INIT;
					osSemaphoreRelease(semaphoreStepperHandle);
				} else {
					on = false;
					//printf("Spegnimento \r\n");
					stepper_rpm = 0;
				}
				break;
			case (101):
					//printf("func/stop \r\n"); // func/stop
					break;
			case (102):
				//printf("Vol+ \r\n");
				if(stepper_rpm <= 13)
					stepper_rpm += 2;
				break;
			case (105):
				//printf("Destra \r\n");
				htim2.Instance->CCR1 = SERVO_INIT + 12;
				osSemaphoreRelease(semaphoreServoSincHandle);
				break;
			case (104):
				//printf("play/pausa \r\n"); //play/pausa
				break;
			case (103):
				//printf("Sinistra \r\n");
				htim2.Instance->CCR1 = SERVO_INIT - 12;
				osSemaphoreRelease(semaphoreServoSincHandle);
				break;
			case (106):
				//printf("Indietro \r\n"); //freccia giù
				stepper_dir = 1;
				break;
			case (107):
				//printf("Vol- \r\n");
				if(stepper_rpm > 2)
					stepper_rpm -= 2;
				break;
			case (108):
				//printf("Avanti \r\n"); //freccia su
				stepper_dir = 0;
				break;
			case (109):
				//printf("EQ \r\n"); //EQ
				break;
			case (110):
				//printf("repeat \r\n"); //rept
				osSemaphoreRelease(semaphoreWifiSincHandle);
				break;
			case (0):
				//printf("0 \r\n"); //0
				break;
			default :
				//printf("Segnale non codificato \r\n");
				break;
	  	}
	  //printf("ACCELERO: X = %d     Y = %d     Z = %d\r\n\n", pDataXYZ[0], pDataXYZ[1], pDataXYZ[2]);
	  osSemaphoreRelease(semaphoreMutexHandle);
  }
}

/* This function is executed in case of error occurrence. */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}


#if defined (TERMINAL_USE)
/**
  * @brief  Retargets the C library TERMOUT function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&hDiscoUart, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}
#endif /* TERMINAL_USE */


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
