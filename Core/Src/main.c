/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"

#include <stdbool.h>
#include <limits.h>

#include "NTC 10K 3989.h"
#include "Full Duplex USART r0.h"
#include "MAX31855 r0.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

volatile typedef struct
{
	uint16_t InputVoltage;
	uint16_t OutputVoltage;
	uint16_t Current;
} tsPowerInformation;
volatile union
{
	struct
	{
		unsigned Powerdown :1;
		unsigned Undervoltage :1;
		unsigned Overvoltage :1;
		unsigned MPQNpg :1;
		unsigned ErrorFanPcb :1;
		unsigned ErrorFan1 :1;
		unsigned Fault :1;
		unsigned ErrorNTCDissipator :1;
		unsigned Dummy :24;
	};
	uint32_t ExistError;
} sFlagsErrors;

volatile struct
{
	tsPowerInformation PowerInformation;
	union
	{
		unsigned CalculateNTC :1;
		unsigned CalculatePD :1;
		unsigned CalculateVBUS :1;
		unsigned Error :5;
		unsigned Dummy :24;
	} Flags;
} sStatusPower;

enum
{
	ERROR_NONE = 0, ERROR_POWERDOWN, ERROR_UNDERVOLTAGE, ERROR_OVERVOLTAGE, ERROR_MPQ_NPG, //NPG = NOT POWER GOOD
	ERROR_FAN_PCB,
	ERROR_FAN1,
	ERROR_FAULT,
	ERROR_NTC_DISSIPATOR,
} eErros;

FD_t* FullDuplexSystem;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ################# DEFINIÇÕES CÓDIGO	 	 ################# */

#define	HIGH			1
#define	LOW				0

/* ################# DEFINIÇÕES AD 		 	 ################# */
#define RES_AD			4095UL			// (2^12 - 1)
#define	VREF			33UL			//3,3V

/* ################# POWERDOWN DEFINIÇÕES 	################# */
#define POWERDOWN_VALUE 160 //16V
#define TIMEOUT_PD		500//ms
#define BUFFER_PD		10
#define RCIMA_PD		10000UL
#define RBAIXO_PD		1240UL
#define PD_MULT			((VREF * (RCIMA_PD + RBAIXO_PD))/((RES_AD * RBAIXO_PD)/PD_DIV))
#define PD_DIV			1000

/* ################# VBUS DEFINIÇÕES 		################# */
#define UNDERVOLTAGE_VALUE 		1000 //100V
#define TIMEOUT_UNDERVOLTAGE	500//ms
#define OVERVOLTAGE_VALUE 		1970 //197V
#define TIMEOUT_OVERVOLTAGE		500//ms
#define LOW_OPERATION	400//40V
#define BUFFER_VBUS		1
#define RCIMA_VBUS		300000UL
#define RBAIXO_VBUS		3900UL
#define VBUS_MULT		((VREF * (RCIMA_VBUS + RBAIXO_VBUS))/((RES_AD * RBAIXO_VBUS) /VBUS_DIV))
#define VBUS_DIV		1000

/* ################# NTC DEFINITIONS ################# */
#define	TEMPERATURE_SHORT_CIRCUIT	1200	//120°C
#define	TEMPERATURE_OPEN_CIRCUIT	-200	//-20°C
#define OVERTEMPERATURE_DISSIPATOR	800		//80°C
#define BUFFER_NTC	1000					//1000 leituras de AD

/* ################# DEFINIÇÕES GERAIS ################# */
#define PERIOD_LED					1000//ms
#define PERIOD_LED_ERROR 			250//ms
#define RELAY_INRUSH				300//ms
#define FAN_PCB_TIMEOUT				5000//ms
#define FAN_1_AND_2_TIMEOUT			5000//ms

/* ################# COEFICIENTE ANGULAR SAIDA  ################# */
#define COEFF_A			1.0
#define COEFF_B			0.0

/* ################# DEFINES MAX31855  ################# */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
uint32_t Adc1Val[2], Adc2Val[3];
volatile uint16_t AdValuePD[BUFFER_PD] =
{ 0 }, AdValueVBUS[BUFFER_VBUS] =
{ 0 }, AdValueNTC[BUFFER_NTC] =
{ 0 };
uint16_t ValuePD = 0, ValueVBUS = 0, ValueNTC = 0;
volatile uint16_t CntTimeoutPD = 0, CntTimeoutOvervoltage = 0, CntTimeoutUndervoltage = 0;
volatile uint8_t CntAdValuePD = 0, CntAdValueVBUS = 0, CntAdValueNTC = 0;
volatile uint16_t CntStatusLED = 0;
volatile uint16_t CntStatusLEDError = 0;
volatile uint16_t Cnt1ms = 0;
volatile uint32_t IC_FanPCB_Value1 = 0, IC_Fan1_Value1 = 0, IC_FanPCB_Value2 = 0, IC_Fan1_Value2 = 0, IC_FanPCB_Difference = 0, IC_Fan1_Difference = 0, IC_FanPCB_Frequency = 0, IC_Fan1_Frequency = 0;
volatile uint8_t IC_FanPCB_IsFirstCaptured = false, IC_Fan1_IsFirstCaptured = false;
volatile uint16_t IC_FanPCB_OvfTimer = 0, IC_Fan1_OvfTimer = 0, IC_FanPCB_RPM = 0, IC_Fan1_RPM = 0;
volatile uint16_t CntFanPCB = 0, CntFan1 = 0, CntRelayInrush = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM16_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM15_Init(void);
/* USER CODE BEGIN PFP */
static void Hw_Buck_Init(void);
static void Hw_Buck_Deinit(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void DMACallbackADC1(DMA_HandleTypeDef *hdma);
void DMACallbackADC2(DMA_HandleTypeDef *hdma);

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
static void StatusLEDControl(uint8_t ErrorCode);
static void SetOutputVoltage(uint16_t OutVoltage);
static void SetError(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_ADC2_Init();
	MX_TIM2_Init();
	MX_TIM16_Init();
	MX_SPI1_Init();
	MX_USART2_UART_Init();
	MX_TIM8_Init();
	MX_TIM1_Init();
	MX_TIM15_Init();
	/* USER CODE BEGIN 2 */
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED); // Calibrate ADC
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED); // Calibrate ADC

	HAL_TIM_Base_Start_IT(&htim2);

	HAL_ADC_Start_DMA(&hadc1, Adc1Val, 2); // Start the ADC in DMA mode
	HAL_ADC_Start_DMA(&hadc2, Adc2Val, 3); // Start the ADC in DMA mode

	hdma_adc1.XferCpltCallback = &DMACallbackADC1;
	hdma_adc2.XferCpltCallback = &DMACallbackADC2;

	HAL_TIM_Base_Start_IT(&htim16);
	HAL_TIM_Base_Start_IT(&htim1);

	HAL_TIM_IC_Start_IT(&htim16, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3); //PWM HIN e LIN
	HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3); // turn on complementary channel

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); //FAN1

	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1); //FAN2

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 0);

	Hw_Buck_Init();
	Init_MAX31855(&hspi1, CS_MAX31855_GPIO_Port, CS_MAX31855_Pin);

	FD_CONFIG_t config;

	config.size_buffer = 50;
	config.uart = &huart2;

	FullDuplexSystem = FDUSART_Init(&config);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		int8_t i = 0;
		uint32_t u32Temp = 0;
		if (sStatusPower.Flags.CalculatePD)
		{
			sStatusPower.Flags.CalculatePD = 0;
			i = 0;
			u32Temp = 0;
			for (i = 0; i < BUFFER_PD; i++)
			{
				u32Temp += AdValuePD[i];
			}
			ValuePD = u32Temp / BUFFER_PD;
			ValuePD = (uint16_t) (((uint32_t) ValuePD * (uint32_t) PD_MULT) / (uint32_t) PD_DIV);
		}
		if (sStatusPower.Flags.CalculateVBUS)
		{

			sStatusPower.Flags.CalculateVBUS = false;
			i = 0;
			u32Temp = 0;
			for (i = 0; i < BUFFER_VBUS; i++)
			{
				u32Temp += AdValueVBUS[i];
			}
			ValueVBUS = u32Temp / BUFFER_VBUS;
			ValueVBUS = (uint16_t) (((uint32_t) ValueVBUS * (uint32_t) VBUS_MULT) / (uint32_t) VBUS_DIV);
			sStatusPower.PowerInformation.InputVoltage = ValueVBUS;
		}
		if (sStatusPower.Flags.CalculateNTC)
		{
			sStatusPower.Flags.CalculateNTC = false;
			i = 0;
			u32Temp = 0;
			for (i = 0; i < BUFFER_NTC; i++)
			{
				u32Temp += AdValueNTC[i];
			}
			ValueNTC = u32Temp / BUFFER_NTC;

			ValueNTC = NTC_TABLE[ValueNTC / 4]; // Divide por 4 pois a tabela é de 1024 pontos e o AD de 4096 --> 4096/1024 = 4

			if (ValueNTC > TEMPERATURE_SHORT_CIRCUIT || ValueNTC < TEMPERATURE_OPEN_CIRCUIT)
			{
				sFlagsErrors.ErrorNTCDissipator = true;
			}
			else
			{
				sFlagsErrors.ErrorNTCDissipator = false;
			}
		}
		SetOutputVoltage(1002);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 16000);
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 16000);

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	RCC_OscInitStruct.PLL.PLLN = 34;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

	ADC_MultiModeTypeDef multimode =
	{ 0 };
	ADC_ChannelConfTypeDef sConfig =
	{ 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.GainCompensation = 0;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.NbrOfConversion = 2;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	hadc1.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_15;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void)
{

	/* USER CODE BEGIN ADC2_Init 0 */

	/* USER CODE END ADC2_Init 0 */

	ADC_ChannelConfTypeDef sConfig =
	{ 0 };

	/* USER CODE BEGIN ADC2_Init 1 */

	/* USER CODE END ADC2_Init 1 */

	/** Common config
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc2.Init.Resolution = ADC_RESOLUTION_12B;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.GainCompensation = 0;
	hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	hadc2.Init.LowPowerAutoWait = DISABLE;
	hadc2.Init.ContinuousConvMode = ENABLE;
	hadc2.Init.NbrOfConversion = 3;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc2.Init.DMAContinuousRequests = ENABLE;
	hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	hadc2.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc2) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_12;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_15;
	sConfig.Rank = ADC_REGULAR_RANK_3;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC2_Init 2 */

	/* USER CODE END ADC2_Init 2 */

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
	hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig =
	{ 0 };
	TIM_MasterConfigTypeDef sMasterConfig =
	{ 0 };
	TIM_IC_InitTypeDef sConfigIC =
	{ 0 };
	TIM_OC_InitTypeDef sConfigOC =
	{ 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig =
	{ 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 9;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 16999;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	__HAL_TIM_DISABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_2);
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

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

	TIM_ClockConfigTypeDef sClockSourceConfig =
	{ 0 };
	TIM_MasterConfigTypeDef sMasterConfig =
	{ 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 1700;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 99;
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
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void)
{

	/* USER CODE BEGIN TIM8_Init 0 */

	/* USER CODE END TIM8_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig =
	{ 0 };
	TIM_MasterConfigTypeDef sMasterConfig =
	{ 0 };
	TIM_OC_InitTypeDef sConfigOC =
	{ 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig =
	{ 0 };

	/* USER CODE BEGIN TIM8_Init 1 */

	/* USER CODE END TIM8_Init 1 */
	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 0;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 1700;
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
	if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
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
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 100;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	__HAL_TIM_DISABLE_OCxPRELOAD(&htim8, TIM_CHANNEL_3);
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 1;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM8_Init 2 */

	/* USER CODE END TIM8_Init 2 */
	HAL_TIM_MspPostInit(&htim8);

}

/**
 * @brief TIM15 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM15_Init(void)
{

	/* USER CODE BEGIN TIM15_Init 0 */

	/* USER CODE END TIM15_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig =
	{ 0 };
	TIM_MasterConfigTypeDef sMasterConfig =
	{ 0 };
	TIM_OC_InitTypeDef sConfigOC =
	{ 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig =
	{ 0 };

	/* USER CODE BEGIN TIM15_Init 1 */

	/* USER CODE END TIM15_Init 1 */
	htim15.Instance = TIM15;
	htim15.Init.Prescaler = 9;
	htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim15.Init.Period = 16999;
	htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim15.Init.RepetitionCounter = 0;
	htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	__HAL_TIM_DISABLE_OCxPRELOAD(&htim15, TIM_CHANNEL_1);
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM15_Init 2 */

	/* USER CODE END TIM15_Init 2 */
	HAL_TIM_MspPostInit(&htim15);

}

/**
 * @brief TIM16 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM16_Init(void)
{

	/* USER CODE BEGIN TIM16_Init 0 */

	/* USER CODE END TIM16_Init 0 */

	TIM_IC_InitTypeDef sConfigIC =
	{ 0 };

	/* USER CODE BEGIN TIM16_Init 1 */

	/* USER CODE END TIM16_Init 1 */
	htim16.Instance = TIM16;
	htim16.Init.Prescaler = 0;
	htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim16.Init.Period = 65535;
	htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
	htim16.Init.RepetitionCounter = 0;
	htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_IC_Init(&htim16) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim16, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM16_Init 2 */

	/* USER CODE END TIM16_Init 2 */

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
	huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMAMUX1_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* DMA1_Channel2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	/* DMA1_Channel3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	/* DMA2_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
	/* DMA2_Channel2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, LED_COMM_Pin | LED_STATUS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, USART_RX_TX_Pin | BUZZER_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CS_MAX31855_GPIO_Port, CS_MAX31855_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(RELAY_INRUSH_GPIO_Port, RELAY_INRUSH_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(FAULT_CLR_GPIO_Port, FAULT_CLR_Pin, GPIO_PIN_SET);

	/*Configure GPIO pins : LED_COMM_Pin LED_STATUS_Pin */
	GPIO_InitStruct.Pin = LED_COMM_Pin | LED_STATUS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : USART_RX_TX_Pin CS_MAX31855_Pin BUZZER_Pin */
	GPIO_InitStruct.Pin = USART_RX_TX_Pin | CS_MAX31855_Pin | BUZZER_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : RELAY_INRUSH_Pin FAULT_CLR_Pin */
	GPIO_InitStruct.Pin = RELAY_INRUSH_Pin | FAULT_CLR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PG_MPQ_Pin FAULT_SD_Pin */
	GPIO_InitStruct.Pin = PG_MPQ_Pin | FAULT_SD_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : FAN2_RPM_Pin SY_FLT_Pin */
	GPIO_InitStruct.Pin = FAN2_RPM_Pin | SY_FLT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/* Pino power good do MPQ */
	if (GPIO_Pin == PG_MPQ_Pin)
	{
		if (HAL_GPIO_ReadPin(PG_MPQ_GPIO_Port, PG_MPQ_Pin) == LOW)
		{
			sFlagsErrors.MPQNpg = true;
		}
		else
		{
			sFlagsErrors.MPQNpg = false;
			SetError(); /* Verify if exist error */
		}
	}

	/* Pin fault */
	if (GPIO_Pin == FAULT_SD_Pin && HAL_GPIO_ReadPin(FAULT_SD_GPIO_Port, FAULT_SD_Pin) == LOW)
	{
		Hw_Buck_Deinit();
		sFlagsErrors.Fault = true;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2)
	{
		InterruptControl_MAX31855();
		SetError(); /* Verify if exist error */
		StatusLEDControl(sStatusPower.Flags.Error);
		if (Cnt1ms > USHRT_MAX - 1)
			Cnt1ms = 0;
		else
			Cnt1ms++;

		/* Power on relay inrush */
		if (CntRelayInrush >= RELAY_INRUSH)
			HAL_GPIO_WritePin(RELAY_INRUSH_GPIO_Port, RELAY_INRUSH_Pin, HIGH);
		else
			CntRelayInrush++;

		/* Erro Fan RPM cooler */
		if (CntFanPCB >= FAN_PCB_TIMEOUT)
		{
			sFlagsErrors.ErrorFanPcb = true;
			IC_FanPCB_RPM = 0;
			IC_FanPCB_Frequency = 0;
		}
		else
		{
			CntFanPCB++;
		}

		/* Erro Fan RPM cooler */
		if (CntFan1 >= FAN_1_AND_2_TIMEOUT)
		{
			sFlagsErrors.ErrorFan1 = true;
			IC_Fan1_RPM = 0;
			IC_Fan1_Frequency = 0;
		}
		else
		{
			if (HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_2) > 0)  //Se o Duty for maior que 0 começa a contar o tempo de timeout
				CntFan1++;
			else
				CntFan1 = 0;
		}

		/* PoweeDown */
		if (ValuePD < POWERDOWN_VALUE)
		{
			if (CntTimeoutPD > TIMEOUT_PD)
			{
				sFlagsErrors.Powerdown = true;
			}
			else
			{
				CntTimeoutPD++;
			}
		}
		else
		{
			sFlagsErrors.Powerdown = false;
			CntTimeoutPD = 0;
		}

		/* Overvoltage, Undervoltage */
		if (sStatusPower.PowerInformation.InputVoltage > OVERVOLTAGE_VALUE)
		{
			if (CntTimeoutOvervoltage > TIMEOUT_OVERVOLTAGE)
			{
				sFlagsErrors.Overvoltage = true;
			}
			else
			{
				CntTimeoutOvervoltage++;
			}
		}

		else if (sStatusPower.PowerInformation.InputVoltage > LOW_OPERATION && sStatusPower.PowerInformation.InputVoltage <= UNDERVOLTAGE_VALUE) // Undervoltage
		{
			if (CntTimeoutUndervoltage > TIMEOUT_OVERVOLTAGE)
			{
				sFlagsErrors.Undervoltage = true;
			}
			else
			{
				CntTimeoutUndervoltage++;
			}
		}
		else
		{
			sFlagsErrors.Undervoltage = false;
			sFlagsErrors.Overvoltage = false;
			CntTimeoutOvervoltage = 0;
			CntTimeoutUndervoltage = 0;
		}
	}
	else if (htim == &htim16)
	{
		if (IC_FanPCB_OvfTimer < USHRT_MAX)
			IC_FanPCB_OvfTimer++;
	}
	else if (htim == &htim1)
	{
		if (IC_Fan1_OvfTimer < USHRT_MAX)
			IC_Fan1_OvfTimer++;
	}
}

void DMACallbackADC1(DMA_HandleTypeDef *hdma)
{
	if (CntAdValueNTC >= BUFFER_NTC)
	{
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		sStatusPower.Flags.CalculateNTC = 1;
		CntAdValueNTC = 0;
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
	}
	else if (sStatusPower.Flags.CalculateNTC == 0)
	{
		AdValueNTC[CntAdValueNTC++] = Adc1Val[1];
	}
}
void DMACallbackADC2(DMA_HandleTypeDef *hdma)
{
	if (CntAdValuePD >= BUFFER_PD)
	{
		sStatusPower.Flags.CalculatePD = 1;
		CntAdValuePD = 0;
	}
	else if (sStatusPower.Flags.CalculatePD == 0)
	{
		AdValuePD[CntAdValuePD++] = Adc2Val[1];
	}
	else
	{
//do nothing
	}
	if (CntAdValueVBUS >= BUFFER_VBUS)
	{
		sStatusPower.Flags.CalculateVBUS = 1;
		CntAdValueVBUS = 0;
	}
	else if (sStatusPower.Flags.CalculateVBUS == 0)
	{
		AdValueVBUS[CntAdValueVBUS++] = Adc2Val[2];
	}
	else
	{
//do nothing
	}
}

static void StatusLEDControl(uint8_t ErrorCode)
{
	CntStatusLED++;

	if (ErrorCode == ERROR_NONE)
	{
		if (CntStatusLED >= PERIOD_LED)
		{
			HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
			CntStatusLED = 0;
		}
	}
	else
	{
		if (CntStatusLED <= PERIOD_LED_ERROR && CntStatusLEDError < ErrorCode - 1)
		{
			HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);
		}
		else if (CntStatusLEDError < ErrorCode - 1)
		{
			HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);

			if (CntStatusLED >= PERIOD_LED_ERROR * 2)
			{
				CntStatusLED = 0;
				CntStatusLEDError++;
			}
		}
		else
		{
			if (CntStatusLED >= PERIOD_LED)
			{
				CntStatusLED = 0;
				CntStatusLEDError = 0;
			}
		}

	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM16) //Fan RPM PCB
	{
		CntFanPCB = 0;
		if (IC_FanPCB_IsFirstCaptured == false)
		{
			IC_FanPCB_OvfTimer = 0;
			IC_FanPCB_Value1 = HAL_TIM_ReadCapturedValue(&htim16, TIM_CHANNEL_1);
			IC_FanPCB_IsFirstCaptured = true;
		}
		else if (IC_FanPCB_IsFirstCaptured == true)
		{
			IC_FanPCB_Value2 = HAL_TIM_ReadCapturedValue(&htim16, TIM_CHANNEL_1);
			if (IC_FanPCB_Value2 > IC_FanPCB_Value1)
			{
				IC_FanPCB_Difference = (IC_FanPCB_Value2 + (IC_FanPCB_OvfTimer * 0xFFFF)) - IC_FanPCB_Value1;
			}
			else if (IC_FanPCB_Value2 < IC_FanPCB_Value1)
			{
				IC_FanPCB_Difference = ((0xFFFF - IC_FanPCB_Value1) + (IC_FanPCB_OvfTimer * 0xFFFF) + IC_FanPCB_Value2) + 1;
			}
			else
			{
				Error_Handler();
			}
			//Frequency = TIM3 CLOCK / Difference
			IC_FanPCB_Frequency = HAL_RCC_GetPCLK1Freq() / IC_FanPCB_Difference;
			IC_FanPCB_RPM = IC_FanPCB_Frequency * 60;
			IC_FanPCB_IsFirstCaptured = false;
		}

	}
	if (htim->Instance == TIM1) //Fan RPM Fan1
	{
		CntFan1 = 0;
		if (IC_Fan1_IsFirstCaptured == false)
		{
			IC_Fan1_OvfTimer = 0;
			IC_Fan1_Value1 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
			IC_Fan1_IsFirstCaptured = true;
		}
		else if (IC_Fan1_IsFirstCaptured == true)
		{
			IC_Fan1_Value2 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
			if (IC_Fan1_Value2 > IC_Fan1_Value1)
			{
				IC_Fan1_Difference = (IC_Fan1_Value2 + (IC_Fan1_OvfTimer * 0xFFFF)) - IC_Fan1_Value1;
			}
			else if (IC_Fan1_Value2 < IC_Fan1_Value1)
			{
				IC_Fan1_Difference = ((0xFFFF - IC_Fan1_Value1) + (IC_Fan1_OvfTimer * 0xFFFF) + IC_Fan1_Value2) + 1;
			}
			else
			{
				Error_Handler();
			}
			//Frequency = TIM3 CLOCK / Difference
			IC_Fan1_Frequency = HAL_RCC_GetPCLK1Freq() / IC_Fan1_Difference;
			IC_Fan1_RPM = IC_Fan1_Frequency * 60;
			IC_Fan1_IsFirstCaptured = false;
		}
	}
}
static void Hw_Buck_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };

	if (HAL_GPIO_ReadPin(FAULT_SD_GPIO_Port, FAULT_SD_Pin) == HIGH)
	{
		HAL_GPIO_WritePin(FAULT_CLR_GPIO_Port, FAULT_CLR_Pin, GPIO_PIN_SET);

		/* Definindo FAULT SD como pino de entrada com interrupção */
		/* Estrutura */
		GPIO_InitStruct.Pin = FAULT_SD_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	}
}
static void Hw_Buck_Deinit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };

	/* Setando Fault Clear para baixo */
	HAL_GPIO_WritePin(FAULT_CLR_GPIO_Port, FAULT_CLR_Pin, GPIO_PIN_RESET);

	/* Definindo FAULT SD como pino de saída e forçando para 0V*/
	/* Estrutura */
	GPIO_InitStruct.Pin = FAULT_SD_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* Forçando FAULT SD para 0V */
	HAL_GPIO_WritePin(FAULT_SD_GPIO_Port, FAULT_SD_Pin, GPIO_PIN_RESET);
}

static void SetOutputVoltage(uint16_t OutVoltage)
{
	uint16_t DutyCalculate = 0;
	uint16_t MaxValueDuty = htim8.Init.Period - 75;

	if (sStatusPower.PowerInformation.InputVoltage > 0)
	{
		DutyCalculate = ((uint32_t) (OutVoltage * htim8.Init.Period) / sStatusPower.PowerInformation.InputVoltage);

		if (OutVoltage > ValueVBUS || DutyCalculate > MaxValueDuty)
			DutyCalculate = MaxValueDuty;

		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, DutyCalculate);
		asm("NOP");
	}

}

static void SetError(void)
{
	if (sStatusPower.Flags.Error == ERROR_NONE)
	{
		if (sFlagsErrors.Fault)
			sStatusPower.Flags.Error = ERROR_FAULT;
		else if (sFlagsErrors.Powerdown)
			sStatusPower.Flags.Error = ERROR_POWERDOWN;
		else if (sFlagsErrors.Undervoltage)
			sStatusPower.Flags.Error = ERROR_UNDERVOLTAGE;
		else if (sFlagsErrors.Overvoltage)
			sStatusPower.Flags.Error = ERROR_OVERVOLTAGE;
		else if (sFlagsErrors.MPQNpg)
			sStatusPower.Flags.Error = ERROR_MPQ_NPG;
		else if (sFlagsErrors.ErrorFanPcb)
			sStatusPower.Flags.Error = ERROR_FAN_PCB;
		else if (sFlagsErrors.ErrorFan1)
			sStatusPower.Flags.Error = ERROR_FAN1;
		else if (sFlagsErrors.ErrorNTCDissipator)
			sStatusPower.Flags.Error = ERROR_NTC_DISSIPATOR;
	}
	else
		Hw_Buck_Deinit();

	if (sFlagsErrors.ExistError == 0) // Se não houver nenhum erro, limpa erro
		sStatusPower.Flags.Error = ERROR_NONE; /* Clear Error */
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
