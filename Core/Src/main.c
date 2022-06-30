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

/* Private includes ----------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "stepper.h"
#include "stm32l475e_iot01.h"
#include "stm32l475e_iot01_tsensor.h"
#include "stm32l475e_iot01_gyro.h"
#include "stm32l475e_iot01_accelero.h"
#include "stm32l475e_iot01_psensor.h"

/* Private typedef -----------------------------------------------------------*/
#define SERVO_INIT 73
#define STEP_DIR_INIT 1
#define STEP_RPM_INIT 8

/* WIFI settings */
#define WIFI_WRITE_TIMEOUT 10000
#define WIFI_READ_TIMEOUT  10000
#define CONNECTION_TRIAL_MAX 10

#define SSID     "Redmi"
#define PASSWORD "BBMito98"
#define RemotePORT	8002

uint8_t RemoteIP[] = {192,168,43,234};
uint8_t  MAC_Addr[6];
uint8_t  IP_Addr[4];
uint8_t wifi_data[70];

int32_t Socket = -1;
int16_t Trials = CONNECTION_TRIAL_MAX;
osEvent retval;
uint16_t Datalen;
int32_t ret;

/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC, small TERMOUT (option LD Linker->Libraries->Small TERMOUT
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

extern SPI_HandleTypeDef hspi;
extern UART_HandleTypeDef hDiscoUart;

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;
UART_HandleTypeDef huart1;

osSemaphoreId semaphoreWifiHandle;
osSemaphoreId semaphoreWifiSincHandle;
osSemaphoreId semaphoreSerialHandle;
osSemaphoreId semaphoreServoPrivHandle;
osSemaphoreId semaphoreServoHandle;
osSemaphoreId semaphoreStepperPrivHandle;
osSemaphoreId semaphoreMutexHandle;
osSemaphoreId semaphoreMutexWifiDataHandle;
osSemaphoreId semaphoreMutexDatasHandle;

osThreadId defaultTaskHandle;
osThreadId ServoTaskHandle;
osThreadId StepperTaskHandle;
osThreadId SendDataTaskHandle;
osThreadId ManagerTaskHandle;
osThreadId SerialTaskHandle;
osThreadId SensorsTaskHandle;
osThreadId WifiTaskHandle;


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
void StartSendData(void const * argument);
void StartManager(void const * argument);
void StartSensors(void const * argument);
void StartWifi(void const * argument);

/* this function generates ACTIVE delay in microseconds --> useful for stepper motor */
void delayStepM (uint16_t us)
{
  __HAL_TIM_SET_COUNTER(&htim8, 0);
  while (__HAL_TIM_GET_COUNTER(&htim8) < us);
}

/* this is an INTERRUPT raised when there something arrives on arduino serial */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	osSemaphoreRelease(semaphoreSerialHandle);
}

/* this function call an interrupt for Wifi */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin) {
    case (GPIO_PIN_1):
    		SPI_WIFI_ISR();
    		break;
    default:
    	break;
  }
}

void SPI3_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi);
}

/* This is an override useful for printf */
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 10);
	return len;
}

/* global variables */
uint8_t stepper_rpm;
bool stepper_dir;

int16_t pDataXYZ[3] = {0};
int temperature;
int pressure;

uint8_t command[1];
bool problem;
bool connection;
bool on;

int main(void)
{
  /* Init STM32 board */
  HAL_Init();
  BSP_LED_Init(LED2);
  MX_USART1_UART_Init();

  /* Configure the system clock */
   SystemClock_Config();

  BSP_ACCELERO_Init();
  BSP_TSENSOR_Init();
  BSP_PSENSOR_Init();

  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();

  /* START TIMERS */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Base_Start(&htim8);

  /* RTOS_SEMAPHORES */
  osSemaphoreDef(semaphoreWifiSinc);
  semaphoreWifiSincHandle = osSemaphoreCreate(osSemaphore(semaphoreWifiSinc), 1);

  osSemaphoreDef(semaphoreWifi);
  semaphoreWifiHandle = osSemaphoreCreate(osSemaphore(semaphoreWifi), 1);

  osSemaphoreDef(semaphoreSerial);
  semaphoreSerialHandle = osSemaphoreCreate(osSemaphore(semaphoreSerial), 1);

  osSemaphoreDef(semaphoreServoPriv);
  semaphoreServoPrivHandle = osSemaphoreCreate(osSemaphore(semaphoreServoPriv), 1);

  osSemaphoreDef(semaphoreStepperPriv);
  semaphoreStepperPrivHandle = osSemaphoreCreate(osSemaphore(semaphoreStepperPriv), 1);

  osSemaphoreDef(semaphoreServo);
  semaphoreServoHandle = osSemaphoreCreate(osSemaphore(semaphoreServo), 1);

  osSemaphoreDef(semaphoreMutex);
  semaphoreMutexHandle = osSemaphoreCreate(osSemaphore(semaphoreMutex), 1);

  osSemaphoreDef(semaphoreMutexDatas);
  semaphoreMutexDatasHandle = osSemaphoreCreate(osSemaphore(semaphoreMutexDatas), 1);

  /* RTOS_THREADS */

  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ServoTask */
  osThreadDef(ServoTask, StartServo, osPriorityNormal, 0, 128);
  ServoTaskHandle = osThreadCreate(osThread(ServoTask), NULL);

  /* definition and creation of StepperTask */
  osThreadDef(StepperTask, StartStepper, osPriorityNormal, 0, 128);
  StepperTaskHandle = osThreadCreate(osThread(StepperTask), NULL);

  /* definition and creation of SendDataTask */
  osThreadDef(SendDataTask, StartSendData, osPriorityNormal, 0, 128);
  SendDataTaskHandle = osThreadCreate(osThread(SendDataTask), NULL);

  /* definition and creation of ManagerTask */
  osThreadDef(ManagerTask, StartManager, osPriorityAboveNormal, 0, 128);
  ManagerTaskHandle = osThreadCreate(osThread(ManagerTask), NULL);

  /* definition and creation of SensorsTask */
  osThreadDef(SensorsTask, StartSensors, osPriorityNormal, 0, 128);
  SensorsTaskHandle = osThreadCreate(osThread(SensorsTask), NULL);

  /* definition and creation of WifiTask */
   osThreadDef(WifiTask, StartWifi, osPriorityNormal, 0, 128);
   WifiTaskHandle = osThreadCreate(osThread(WifiTask), NULL);

  /* Init variables */
  HAL_UART_Receive_IT(&huart3, command, 1);
  problem = false;
  connection = false;
  on = false;

  /* Start scheduler */
  osKernelStart();

  while (1) {}
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
	  //printf("DO NOTHING \r\n");
  }
}

/* USER CODE BEGIN Header_StartServo */
void StartServo(void const * argument)
{
	htim2.Instance->CCR1 = SERVO_INIT;
	for(;;) {
		/* Inizio sezione critica.
		 * Le variabili condivise sono htim2.Instance->CCR1 e ON. */
		osSemaphoreWait(semaphoreMutexHandle, osWaitForever);

		/* Il semaforo ServoPriv è un semaforo privato.
		 * Se le condizioni sono vere allora il semaforo verrà incrementato e la successiva wait sarà passante.
		 * Se una delle condizioni è false allora il semaforo non verrà incrementato e la successiva wait sarà bloccante. */
		if ((htim2.Instance->CCR1 != SERVO_INIT) && (on == true)) {
			if (htim2.Instance->CCR1 > SERVO_INIT)
				htim2.Instance->CCR1 -=2;
			else
				htim2.Instance->CCR1 +=2;

			osSemaphoreRelease(semaphoreServoPrivHandle);
		}

		osSemaphoreRelease(semaphoreMutexHandle); //fine sezione critica
		osSemaphoreWait(semaphoreServoPrivHandle, osWaitForever);
		osDelay(50);
	}
}

/* USER CODE BEGIN Header_StartStepper */
void StartStepper(void const * argument)
{
  for(;;)
  {
	  /* Inizio sezione critica.
	   * Le variabili condivise sono PROBLEM, ON, DIR, RPM. */

	  osSemaphoreWait(semaphoreMutexHandle, osWaitForever);

	  /* Il semaforoStepperPriv è un semaforo privato.
	   * Se è tutto ok allora verrà incrementato in modo che la wait successiva sarà passante.
	   * Se non è tutto ok allora il semaforo non verra incrementato quindi la wait successiva sarà bloccante. */
	  if((problem == false) && (on == true)){
		  stepper_step_angle(10, stepper_dir, stepper_rpm);
		  osSemaphoreRelease(semaphoreStepperPrivHandle);
	  }
	  osSemaphoreRelease(semaphoreMutexHandle); // fine sezione critica
	  osSemaphoreWait(semaphoreStepperPrivHandle, osWaitForever);
	  osDelay(20);
  }
}

/* USER CODE BEGIN Header_StartWifi */
void StartWifi(void const * argument)
{
	for(;;) {

		/* Il threadWifi resta bloccato fin quando la connessione è già instaurata. Per sbloccarlo
		 * deve essere cliccato il tasto opportuno quando la connessione non è attiva. */

		while (connection == true)
			osSemaphoreWait(semaphoreWifiSincHandle, osWaitForever);

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

  	              /* Quando la connessione viene instaurata correttamente viene fatto partire il threadSendData */
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

/* USER CODE BEGIN Header_StartSendData */
void StartSendData(void const * argument)
{
  	 for (;;)
  	 {
  		 /* Se non c'è connessione è inutile inviare dati al server quindi il threadSendData si blocca */
  		 if (connection == false) {
  			 osSemaphoreWait(semaphoreWifiHandle, osWaitForever);
  		 }
  		 if(Socket != -1)
  		 {
  			 osSemaphoreWait(semaphoreMutexDatasHandle, osWaitForever);
	         ret = WIFI_SendData(Socket, (uint8_t*) wifi_data, 70, &Datalen, WIFI_WRITE_TIMEOUT);
	         osSemaphoreRelease(semaphoreMutexDatasHandle);
	         if (ret != WIFI_STATUS_OK)
	         {
	        	 printf("\r\n> ERROR : Failed to Send Data, connection closed\r\n");
	        	 connection = false;
	         }
  		 }
  		osDelay(3000);
  	 }
}


/* USER CODE BEGIN Header_StartSensors */
void StartSensors(void const * argument)
{
  for(;;)
  {
	  BSP_ACCELERO_AccGetXYZ(pDataXYZ);
	  temperature = (int) BSP_TSENSOR_ReadTemp();
	  pressure = (int) BSP_PSENSOR_ReadPressure();
	  osSemaphoreWait(semaphoreMutexDatasHandle, osWaitForever);
	  sprintf(wifi_data, "Temperatura: %d \r\nPressione: %d \r\nInclinazione: %d \r\n\n", temperature, pressure, pDataXYZ[0]);
	  osSemaphoreRelease(semaphoreMutexDatasHandle);
	  //osMessagePut(QueueDataHandle, (uint32_t) &wifi_data, 200);

	  /* Sezione critica protetta da mutex.
	   * Le variabile condivisa è PROBLEM */

	  osSemaphoreWait(semaphoreMutexHandle, osWaitForever);

	  if ((pDataXYZ[0] < -200) || (pDataXYZ[0] > 200) || (pDataXYZ[3] < 0) || (temperature > 50)) {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		  problem = true;
	  } else {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		  problem = false;

		  /* Se il problema viene risolto viene fatto ripartire il motore stepper */
		  osSemaphoreRelease(semaphoreStepperPrivHandle);
	  }

	  osSemaphoreRelease(semaphoreMutexHandle); // fine sezione critica
	  osDelay(500);
  }
}

/* USER CODE BEGIN Header_StartManager */
void StartManager(void const * argument)
{
  for(;;)
  {
	  /* semaforo su cui si blocca il threadManager in attesa di arrivo di comandi su seriale arduino */
	  osSemaphoreWait(semaphoreSerialHandle, osWaitForever);
	  HAL_UART_Receive_IT(&huart3, command, 1);

	  /* Inizio sezione critica protetta da mutex. L
	   * Le variabili condivise con altri thread sono RPM, DIR, ON, htim2.Instance->CCR1 */

	  osSemaphoreWait(semaphoreMutexHandle, osWaitForever);
	  switch (command[0]) {
			case (100):
				if(stepper_rpm == 0) {
					//printf("Accensione \r\n");
					on = true;
					stepper_rpm = STEP_RPM_INIT;

					/* Al click del tasto di accensione viene fatto partire il thread che fa ruotare il motore stepper */
					osSemaphoreRelease(semaphoreStepperPrivHandle);
				} else {
					//printf("Spegnimento \r\n");
					on = false;
					stepper_rpm = 0;
				}
				break;
			case (101):
				//printf("func/stop \r\n");
				break;
			case (102):
				//printf("Vol+ \r\n");
				if(stepper_rpm <= 13)
					stepper_rpm += 2;
				break;
			case (105):
				//printf("Destra \r\n");
				if (on == true)
					htim2.Instance->CCR1 = SERVO_INIT + 14;

				/* Quando le ruote curvano viene fatto partire il thread che si occupa di riaddrizzarle */
				osSemaphoreRelease(semaphoreServoPrivHandle);
				break;
			case (104):
				//printf("play/pausa \r\n");
				break;
			case (103):
				//printf("Sinistra \r\n");
				if (on == true)
					htim2.Instance->CCR1 = SERVO_INIT - 14;

				/* Quando le ruote curvano viene fatto partire il thread che si occupa di riaddrizzarle */
				osSemaphoreRelease(semaphoreServoPrivHandle);
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
				//printf("EQ \r\n");
				break;
			case (110):
				//printf("repeat \r\n");

				/* Al click del tasto viene fatto partire il thread che instaura la connessione con il server TCP */
				osSemaphoreRelease(semaphoreWifiSincHandle);
				break;
			case (0):
				//printf("0 \r\n");
				break;
			default :
				//printf("Segnale non codificato \r\n");
				break;
	  	}
	  osSemaphoreRelease(semaphoreMutexHandle); // Fine sezione critica
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
