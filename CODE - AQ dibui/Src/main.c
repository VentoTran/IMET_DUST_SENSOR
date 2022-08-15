/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal.h"
#include "stdio.h"
#include "string.h"
#include "Waveshare_SIM7600.h"
#include "uart.h"
#include "flash.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BQ_ADDR			0xD0
#define PROGRAM_START_DP 		 	 (uint32_t)0x0801E000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_IWDG_Init(void);
static void MX_ADC1_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
 /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
 #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
 #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
uint8_t BCD2DEC(uint8_t data)
{
	return (data>>4)*10 + (data&0x0F);
}
 
uint8_t DEC2BCD(uint8_t data)
{
	return (data/10)<<4 | (data%10);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t time_data[7], rx3_data, rx2_data,  rx2_done = 0, read_ms = 0;
uint8_t DataRs485, Rs485Index = 0, Data_MB[99], Index_MB;
uint16_t rx2_index = 0, Index_SIM = 0, PM_data[4], adc_AI[2];
char Pos_Lat[15], Pos_Log[15];
char  Sim_Data[668], Sim_MS[668], Sim_res[200];
char aux_string[200], str_data[300], str_time[20], file_name[60], TCP_Message[15];

uint32_t sumbyte_check = 0, sum_byte = 0, time_run, time_ms, time_get, time_led, time_dt, time_of4g, TIME_FR, TIME_UP = 12000, TIME_4G = 5000;
uint32_t SPS30_DT[4], SCD_DT[3], time_net, time_FTP;
float temp, humi, ppm_CO2, PM_x[4], V_bat;
uint8_t time_data1[7], get_time = 0, id_sim, time_gps = 0, on_4g = 0, en_w = 1, net_ok = 0, send_dt = 0, id_get = 0, time_dw = 0, 
      up_fr = 0, idpage = 0, page = 0, cfg_SCD30[5] = {0x00,0x01,0x00,0x00,0x81}, read_SCD30[2] = {0x03,0x00}, data_SCD30[18], Mode_SPS30[8];
char TIME_RTC[6][3];
char ServerIP[] = "27.72.59.208", Port[] = "1111";
//char ServerIP[] = "4.tcp.ngrok.io", Port[] = "13310";
//char FTPServer[] = "103.1.210.21", FTPPort[] = "21", FTPUserName[] = "Lam", FTPPassWord[] = "1";				
uint16_t CheckSumCrc16(uint8_t *ucBufferTemp, uint8_t ucLength)
{
  uint32_t CRCFull = 0xFFFF;
  char CRCLSB;
  for ( uint8_t i = 0; i < ucLength; i++ )
  {
		CRCFull = (uint16_t) (CRCFull ^ ucBufferTemp[i]);
    for ( int j = 0; j < 8; j++ )
    {
			CRCLSB = (char) (CRCFull & 0x0001);
      CRCFull = (uint16_t) ((CRCFull >> 1) & 0x7FFFF);
            
      if ( CRCLSB == 1 )
      {
				CRCFull = (uint16_t) (CRCFull ^ 0xA001);
      }
    }
  }
  return CRCFull;
}				
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == NET_Pin) {
		//printf("time: %d", HAL_GetTick()-time_net);
		time_net = HAL_GetTick();
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	 if(htim->Instance == TIM3)
	 {
		 Index_MB = Rs485Index;
//		 printf("\n\r");
//		 for(uint8_t i=0; i<Rs485Index; i++) {
//			  printf("%d  ", Data_MB[i]);//UART_putCharPC(DatabufferRs485[i]);
//		 }
		 //printf("ind: %d\n\r", Index_MB);
		 HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		 //for(uint8_t i=0; i<Index_MB; i++)  Data_MB[i] = DatabufferRs485[i];
		 //memset(DatabufferRs485, '\0', 255);
		 //if(Rs485Index > 98) HAL_UART_Receive_IT(&huart4, &DataRs485, 1);
		 Rs485Index = 0; 
		 HAL_TIM_Base_Stop_IT(&htim3);
	 }
	 if(htim->Instance == TIM2)
	 {
		 Index_SIM = rx2_index; //printf("Anh Trung: %d\n\r", Index_SIM);
		 if(read_ms) printf("%s", Sim_Data);//UART_putStringPC(Sim_res);
		 rx2_index = 0;  rx2_done = 0; read_ms = 1; 
		 HAL_TIM_Base_Stop_IT(&htim2);
	 }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3){
		if(rx3_data == '^') NVIC_SystemReset();
		//HAL_UART_Receive_IT(&huart3, &rx3_data, 1);
	}
	if(huart->Instance == UART4){
		Data_MB[Rs485Index++] = DataRs485;// UART_putCharPC(DataRs485);
		DataRs485 = 0;
    //Rs485Index++;	
		if(Rs485Index == 2) HAL_TIM_Base_Start_IT(&htim3);
		__HAL_TIM_SET_COUNTER(&htim3, 0);
		//if(Rs485Index < 255) HAL_UART_Receive_IT(&huart4, &DataRs485, 1);
		HAL_UART_Receive_IT(&huart4, &DataRs485, 1);
	}
	if(huart->Instance == USART2){
		Sim_Data[rx2_index++] = rx2_data;
		//if(rx2_done == 0) rx2_done = 2;
		rx2_data = 0;
		if(rx2_index == 1) HAL_TIM_Base_Start_IT(&htim2);
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		if(rx2_index < 668) HAL_UART_Receive_IT(&huart2, &rx2_data, 1);
		
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
//	 SCB->VTOR = (uint32_t)0x08003000;
//  __enable_irq();
	//MX_IWDG_Init(); 
//	IWDG->KR = 0xAAAA; // Writing 0xAAAA in the Key register prevents watchdog reset
//	IWDG->KR = 0xCCCC; // Start the independent watchdog timer
	//HAL_Delay(500);
//	hiwdg.Init.Reload = 100;
//	HAL_IWDG_Init(&hiwdg);
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
  MX_I2C1_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_SPI2_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_IWDG_Init();
  MX_ADC1_Init();
  MX_UART4_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	//SIM_PowerOff();
	SIM_PowerOn();
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 50);
	HAL_TIM_Base_Stop_IT(&htim2);
	HAL_TIM_Base_Stop_IT(&htim3);
	HAL_UART_Receive_IT(&huart3, &rx3_data, 1);
	HAL_UART_Receive_IT(&huart2, &rx2_data, 1);
	HAL_UART_Receive_IT(&huart4, &DataRs485, 1);
	HAL_GPIO_WritePin(CTR_PWETH_GPIO_Port, CTR_PWETH_Pin, GPIO_PIN_SET); //tat nguon ethernet
	HAL_GPIO_WritePin(WP_FLH_GPIO_Port, WP_FLH_Pin, GPIO_PIN_RESET);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_AI, 2);
	HAL_GPIO_WritePin(RST_FLH_GPIO_Port, RST_FLH_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CTR_SIM1_GPIO_Port, CTR_SIM1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CTR_SIM2_GPIO_Port, CTR_SIM2_Pin, GPIO_PIN_SET);
	//SIM_PowerOn();
	//HAL_GPIO_WritePin(SIM_PWON_GPIO_Port, SIM_PWON_Pin, GPIO_PIN_SET);
	printf("Anh Trung\n\r");
	

	if(HAL_I2C_IsDeviceReady(&hi2c1, BQ_ADDR, 1, 100)==0) printf("DS1307 is OK\n\r");
	HAL_I2C_Master_Transmit(&hi2c1, 0xC2, cfg_SCD30, 5, 100);
	MX_IWDG_Init();
	hiwdg.Init.Reload = 4000;
	HAL_IWDG_Init(&hiwdg);
	IWDG->KR = 0xAAAA; // Writing 0xAAAA in the Key register prevents watchdog reset
	IWDG->KR = 0xCCCC; // Start the independent watchdog timer
	//USER_initialize(SDFatFs.drv);
	uint32_t time1s = HAL_GetTick(); 
	sendATcommand("AT"); 
	if(HAL_GetTick()-time1s < 910){
		sendATcommand("AT+NETOPEN");   
		sendATcommand("AT+CATR=1");		
		TIME_4G = 10000;
		on_4g = 0; net_ok = 1;
	}
	Mode_SPS30[0] = 0x7E; Mode_SPS30[1] = 0x00; Mode_SPS30[2] = 0x01; Mode_SPS30[3] = 0x00; Mode_SPS30[4] = 0xFE; 
	Mode_SPS30[5] = 0x7E; //Mode_SPS30[6] = 0x00; Mode_SPS30[7] = 0x00;
	memset(Data_MB, '\0', 99);
	HAL_UART_Transmit(&huart4, Mode_SPS30, 6, 100);
	HAL_Delay(2000);
	Mode_SPS30[0] = 0x7E; Mode_SPS30[1] = 0x00; Mode_SPS30[2] = 0x00; Mode_SPS30[3] = 0x02; Mode_SPS30[4] = 0x01; 
	Mode_SPS30[5] = 0x03; Mode_SPS30[6] = 0xF9; Mode_SPS30[7] = 0x7E;
	memset(Data_MB, '\0', 99);
	HAL_UART_Transmit(&huart4, Mode_SPS30, 8, 100);
	HAL_UART_Receive_IT(&huart4, &DataRs485, 1);
	time_run = time_ms = time_of4g = time_led = time_dt = time_get = time_net = HAL_GetTick();
	//static int cnt = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {	
		IWDG->KR = 0xAAAA;
		if(HAL_GetTick()-time_of4g > TIME_4G){
			IWDG->KR = 0xAAAA;
			switch (on_4g)
			{
				case 4:
					Network_check(); PDPSetting();		
					sendATcommand("AT+CIPRXGET=1"); 
					sendATcommand("AT+CIPSRIP=0");
					printf("SIM7600 is OK\n\r"); 
					TIME_4G = 10000; on_4g = 0; //send_dt = 1;
					net_ok = 1; 
					//send_dt = 1;
					time_net = time_ms = HAL_GetTick(); 
					break;
				case 3:
					HAL_GPIO_WritePin(SIM_PWON_GPIO_Port, SIM_PWON_Pin, GPIO_PIN_RESET);
					TIME_4G = 30000; on_4g = 4;
					break;
				case 2:
					HAL_GPIO_WritePin(SIM_PWON_GPIO_Port, SIM_PWON_Pin, GPIO_PIN_SET);
					TIME_4G = 400;  on_4g = 3;
					break;
				case 1:
					SIM_PowerOn(); 
					TIME_4G = 900;  on_4g = 2;
					break;
				case 0:
					if(HAL_GetTick()-time_net > 3000){
						SIM_PowerOff();
						on_4g = 1; net_ok = 0; TIME_4G = 2000;
					}
					else { TIME_4G = 10000; }
					break;
			}
			time_of4g = HAL_GetTick();
		}
		
		if(HAL_GetTick()-time_ms > 10000){
			uint16_t CRCFull = 0xFFFF;
			TCP_Message[0] = 1; TCP_Message[1] = 8;  TCP_Message[12] = 254;
			for(uint8_t i=0; i<4; i++){
				TCP_Message[2+2*i] = (uint8_t) (PM_data[i] >> 8); TCP_Message[3+2*i] = (uint8_t) PM_data[i];  
				//TCP_Message[4+3*i] = ';';
			}
			CRCFull = CheckSumCrc16((uint8_t *)TCP_Message, 10);
			TCP_Message[10] = (uint8_t) (CRCFull & 0xFF);
			TCP_Message[11] = (uint8_t) ((CRCFull >> 8) & 0xFF);
			if(send_dt) TCPClientSendMS(TCP_Message, 13);
			if(net_ok) {
				TCPClient(ServerIP, Port);
				send_dt = 1; net_ok = 0;
			}
			time_ms = HAL_GetTick();	
		}
		if(HAL_GetTick()-time_dt > 3550){
			if((Index_MB == 47)&&(Data_MB[0] == 126)&&(Data_MB[Index_MB-1] == 126)&&(Data_MB[1] == Data_MB[3])&&(Data_MB[4] == 40)){
				for(uint8_t i=0; i<4; i++){
					SPS30_DT[i] = (uint32_t)(Data_MB[4*i+5] << 24) | (uint32_t)(Data_MB[4*i+6] << 16) | (uint32_t)(Data_MB[4*i+7] << 8) | (uint32_t)Data_MB[4*i+8];
					PM_x[i] = *(float*)&SPS30_DT[i]; 
					PM_data[i] = (uint16_t)(100*PM_x[i]);
				}
				printf("PM1.0: %.2f ug/m3\n\rPM2.5: %.2f ug/m3\n\rPM4.0: %.2f ug/m3\n\rPM10: %.2f ug/m3\n\r", PM_x[0], PM_x[1], PM_x[2], PM_x[3]);
				V_bat = adc_AI[1]*3.3*4.03/4095;
				printf("ADC: %d   %d    %.2f Volt\n", adc_AI[0], adc_AI[1], V_bat);
			}
			Mode_SPS30[0] = 0x7E; Mode_SPS30[1] = 0x00; Mode_SPS30[2] = 0x03; Mode_SPS30[3] = 0x00; Mode_SPS30[4] = 0xFC; 
			Mode_SPS30[5] = 0x7E; //Mode_SPS30[6] = 0x00; Mode_SPS30[7] = 0x00;
			memset(Data_MB, '\0', 99);
			HAL_UART_Transmit(&huart4, Mode_SPS30, 6, 100);
			//HAL_UART_Transmit( &huart3, Mode_SPS30, 8, 100 );
			HAL_UART_Receive_IT(&huart4, &DataRs485, 1);
			time_dt = HAL_GetTick();
		}
		if(HAL_GetTick()-time_run > 300){
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
			time_run = HAL_GetTick();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL15;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_2;
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
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 4000;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 59;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 59999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 599;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 11999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 59;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 59999;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 100;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, SIM_PWON_Pin|SIM_RST_Pin|CTR_SIM2_Pin|CTR_PWETH_Pin
                          |LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CTR_SIM1_Pin|WP_FLH_Pin|RST_FLH_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, FLASH_CS_Pin|DE_Pin|OUT2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED2_Pin|GPIO_PIN_12|GPIO_PIN_3|CTR_PW4G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CTR_GND4G_GPIO_Port, CTR_GND4G_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : SIM_PWON_Pin SIM_RST_Pin CTR_SIM1_Pin CTR_SIM2_Pin
                           WP_FLH_Pin RST_FLH_Pin CTR_PWETH_Pin LED1_Pin */
  GPIO_InitStruct.Pin = SIM_PWON_Pin|SIM_RST_Pin|CTR_SIM1_Pin|CTR_SIM2_Pin
                          |WP_FLH_Pin|RST_FLH_Pin|CTR_PWETH_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : FLASH_CS_Pin DE_Pin OUT2_Pin */
  GPIO_InitStruct.Pin = FLASH_CS_Pin|DE_Pin|OUT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin PB12 PB3 CTR_PW4G_Pin
                           CTR_GND4G_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|GPIO_PIN_12|GPIO_PIN_3|CTR_PW4G_Pin
                          |CTR_GND4G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_ETH_Pin */
  GPIO_InitStruct.Pin = INT_ETH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT_ETH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN2_Pin IN1_Pin */
  GPIO_InitStruct.Pin = IN2_Pin|IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : NET_Pin */
  GPIO_InitStruct.Pin = NET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NET_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE 
{
 /* Place your implementation of fputc here */
 /* e.g. write a character to the USART */
 HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 100);
 return ch;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
