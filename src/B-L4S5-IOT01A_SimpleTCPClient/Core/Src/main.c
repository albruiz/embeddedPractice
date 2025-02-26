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
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "wifi.h"
#include "stm32l4s5i_iot01.h"
#include "stm32l4s5i_iot01_tsensor.h"
#include "stm32l4s5i_iot01_hsensor.h"
#include <stdlib.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE_HELLO_MSG         50
#define BUFFER_SIZE_SENSORS_UPDATE    25    // 1 + 4*6 = 25 bytes
#define BUFFER_SIZE_TIMER_UPDATE      5     // 1 + 4 = 5
#define BUFFER_SIZE_THRESHOLD_UPDATE  13    // 1 + 4*3 = 13 bytes
#define DEFAULT_TEMP                  25.0f
#define DEFAULT_HUM                   40.0f
#define DEFAULT_LIGHT                 40.0f
#define DEFAULT_TEMP_THRE             30.0f
#define DEFAULT_HUM_THRE              60.0f
#define DEFAULT_LIGHT_THRE            40.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#if (defined(__GNUC__) && !defined(__CC_ARM))
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

OSPI_HandleTypeDef hospi1;

RNG_HandleTypeDef hrng;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
extern  SPI_HandleTypeDef hspi;
char pcWifiModuleName[100];
char pcWifiModuleId[100];
char pcWifiModuleFwRev[100];
uint8_t pu8WifiModuleMacAddress[6];
uint8_t pu8LocalIpv4[4];
uint8_t pu8RemoteIpv4[4];
uint8_t pu8TxData [500];
uint8_t pu8RxData [500];
int iSendDataLength;
int iReceivedDataLength;

float gTemperature = DEFAULT_TEMP;  // global temperature variable
float gHumidity = DEFAULT_HUM;      // global humidity variable
float gLight = DEFAULT_LIGHT;       // global light level variable

float gTempThreshold = DEFAULT_TEMP_THRE;   // global temperature variable
float gHumThreshold = DEFAULT_HUM_THRE;     // global humidity threshold variable
float gLightThreshold = DEFAULT_LIGHT_THRE; // global light level threshold variable

bool gTempAlarm = false;  // global temperature alarm flag
bool gHumAlarm = false;   // global humidity alarm flag
bool gLightAlarm = false; // global light level alarm

// Structures declarations
typedef enum {
    MESSAGE_TYPE_HELLO = 0,
    MESSAGE_TYPE_UPDATE,
    MESSAGE_TYPE_TIMER_UPDATE,
    MESSAGE_TYPE_THRESHOLD_UPDATE,
    MESSAGE_TYPE_SENSORS_REQUEST,
    MESSAGE_TYPE_ALARM,
//    MESSAGE_TYPE_THRESHOLD_HUM,
//    MESSAGE_TYPE_THRESHOLD_LIGHT
} MessageType;

typedef struct {
    uint8_t type;  // MessageType (defined above)
    char message[BUFFER_SIZE_HELLO_MSG]; // "Hello from web" message
} HelloMessage;

typedef struct {
    uint8_t type;       // MessageType
    uint32_t timer_value;  // New timer value (seconds)
} TimerUpdateMessage;

typedef struct {
    uint8_t type;       // MessageType
    float temperature_threshold;
    float humidity_threshold;
    float light_threshold;
} ThresholdUpdateMessage;

typedef struct {
    uint8_t type;  // 0 to 2 (3 possible values) 0=UpdateNow, 1=SetUpTimer, 2=SetUpThresholds
    uint8_t time;  // 0 to 3 (4 possible values) (def)0=30s, 1=1min, 2=5min, 3=30min
    float temp;    // Temperature (def 20.0)
    float hum;     // Humidity (def 40.0)
    float light;   // Light level (def 100.0)
} messageReceived;

#pragma pack(push, 1)
typedef struct {
	uint8_t type;  // 0 to 1 (2 possible values) 0=normalUpdateMessage, 1=warning, 2=shuttersBeingClosed
	float temp;    // Temperature (can be negative) (def 20.0)
	float hum;     // Humidity (def 40.0)
	float light;   // Light level (def 100.0)
	float temperature_threshold;
  float humidity_threshold;
  float light_threshold;
} mesageSent;
#pragma pack(pop)

// Wi-Fi related functions declarations
messageReceived parseMessage(const char *rxData);
WIFI_Status_t sendMessage(const mesageSent* message);
//WIFI_Status_t sendThresholds();
uint8_t* packMessage(const mesageSent* message, uint32_t* packedSize);

// Sensors related functions declarations
void processReceivedData(uint8_t* data, int dataLength);
uint16_t readLightLevelValues();
void readSensorValues();
void checkThresholds();
void sendData(int type);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_OCTOSPI1_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_USB_Init(void);
static void MX_RNG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void){
  /* USER CODE BEGIN 1 */
  WIFI_Ecn_t  enRoutreEncryptiontype = WIFI_ECN_WPA2_PSK; // set your Router encryption
  uint16_t port = 48569;

  /* Configuracion de Alberto */
//  char pcRouterSSID[] = "MOVISTAR_D0F0"; // Alberto
//  char pcRouterPWR[] = "faGAEandMxjdVvMwAqJa"; // Alberto
//  pu8RemoteIpv4[0] = 192;
//  pu8RemoteIpv4[1] = 168;
//  pu8RemoteIpv4[2] = 1;
//  pu8RemoteIpv4[3] = 35;

  /* Configuracion de Carlos */
	char pcRouterSSID[] = "2-1-M11"; // Carlos
  char pcRouterPWR[] = "Mercader19012768"; // Carlos
  pu8RemoteIpv4[0] = 192;
  pu8RemoteIpv4[1] = 168;
  pu8RemoteIpv4[2] = 31;
  pu8RemoteIpv4[3] = 12;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_DFSDM1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_OCTOSPI1_Init();
  MX_SPI1_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_USB_Init();
  BSP_TSENSOR_Init(); // Initialize Temperature sensor
  BSP_HSENSOR_Init(); // Initialize Humidity sensor
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */

  // WiFi module initialization
  printf("*  WiFi module initialization *\n\r");
  if(WIFI_Init() == WIFI_STATUS_OK){
	  printf("WIFI initialization success\n\n\r");
	  printf("************************ WiFi Module infos ************************\n\n\r");
	  // get module name
	  if(WIFI_GetModuleName(pcWifiModuleName) == WIFI_STATUS_OK){
		  printf("Wifi Module Name: %s\n\r",pcWifiModuleName);
	  }else{
		  printf("!! couldn't get Wifi module name\n\r");
	  }
	  // get module ID
	  if(WIFI_GetModuleID(pcWifiModuleId) == WIFI_STATUS_OK){
		  printf("Wifi Module ID: %s\n\r",pcWifiModuleId);
	  }else{
		  printf("!! couldn't get Wifi module ID\n\r");
	  }
	  // get module Firmware revision
	  if(WIFI_GetModuleFwRevision(pcWifiModuleFwRev) == WIFI_STATUS_OK){
		  printf("Wifi Module Firmware revision: %s\n\r",pcWifiModuleFwRev);
	  }else{
		  printf("!! couldn't get Wifi module Firmware revision\n\r");
	  }
	  // get module Mac@
	  if(WIFI_GetMAC_Address(pu8WifiModuleMacAddress) == WIFI_STATUS_OK){
		  printf("Wifi Module MAC address: %02X-%02X-%02X-%02X-%02X-%02X\n\r",
				  pu8WifiModuleMacAddress[0],pu8WifiModuleMacAddress[1],
				  pu8WifiModuleMacAddress[2],pu8WifiModuleMacAddress[3],
				  pu8WifiModuleMacAddress[4],pu8WifiModuleMacAddress[5]);
	  }else{
		  printf("!! couldn't get Wifi module MAC address\n\r");
	  }
	  printf("*******************************************************************\n\r");

	  // Connection to AP
	  if(WIFI_Connect(pcRouterSSID, pcRouterPWR, enRoutreEncryptiontype) == WIFI_STATUS_OK){
		  printf("Successfully connected to router %s\n\r", pcRouterSSID);
		  if(WIFI_GetIP_Address(pu8LocalIpv4) == WIFI_STATUS_OK){ // get ip address
			  printf("Device IPv4: %u.%u.%u.%u\n\r", pu8LocalIpv4[0],pu8LocalIpv4[1], pu8LocalIpv4[2],pu8LocalIpv4[3]);
		  }

		  // Open TCP client
		  if(WIFI_OpenClientConnection(0, WIFI_TCP_PROTOCOL, "TCP_CLIENT", pu8RemoteIpv4, port , 0) == WIFI_STATUS_OK){
			  printf("TCP client successfully created. \n\r");
			  sendData(MESSAGE_TYPE_UPDATE); // send initial data.
		  }else{
			  printf("!! Couldn't create TCP client. Review WiFi settings and re-run. \n\r");
			  return -1;
		  }
	  }else{
		  printf("!! Couldn't connect to router. Review WiFi settings and re-run. \n\r");
		  return -2;
	  }
  }else{
	  printf("!! WIFI initilization failed. Review WiFi settings and re-run. \n\r");
	  return -3;
  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
    /* USER CODE END WHILE */

    readSensorValues();
    checkThresholds();

    WIFI_Status_t receive_status = WIFI_ReceiveData(0, pu8RxData, sizeof(pu8RxData), &iReceivedDataLength, 5000);
	  if(receive_status == WIFI_STATUS_OK){
		  if(iReceivedDataLength>0){
			  //with'/0' set the new message end, in case the new message length is lower than the old message
			  pu8RxData[iReceivedDataLength] = '\0';

			  printf("Received %d bytes of data.\n\r", iReceivedDataLength);
        processReceivedData(pu8RxData, iReceivedDataLength); // Process the received data
		  }
	  }
    //sendData(MESSAGE_TYPE_UPDATE);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_RNG
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_channel2.Instance = DFSDM1_Channel2;
  hdfsdm1_channel2.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel2.Init.OutputClock.Divider = 2;
  hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel2.Init.Awd.Oversampling = 1;
  hdfsdm1_channel2.Init.Offset = 0;
  hdfsdm1_channel2.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

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
  hi2c1.Init.Timing = 0x307075B1;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x307075B1;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief OCTOSPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI1_Init(void)
{

  /* USER CODE BEGIN OCTOSPI1_Init 0 */

  /* USER CODE END OCTOSPI1_Init 0 */

  OSPIM_CfgTypeDef OSPIM_Cfg_Struct = {0};

  /* USER CODE BEGIN OCTOSPI1_Init 1 */

  /* USER CODE END OCTOSPI1_Init 1 */
  /* OCTOSPI1 parameter configuration*/
  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThreshold = 1;
  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_MACRONIX;
  hospi1.Init.DeviceSize = 32;
  hospi1.Init.ChipSelectHighTime = 1;
  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi1.Init.ClockPrescaler = 1;
  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
  hospi1.Init.ChipSelectBoundary = 0;
  hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
  if (HAL_OSPI_Init(&hospi1) != HAL_OK)
  {
    Error_Handler();
  }
  OSPIM_Cfg_Struct.ClkPort = 1;
  OSPIM_Cfg_Struct.NCSPort = 1;
  OSPIM_Cfg_Struct.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
  if (HAL_OSPIM_Config(&hospi1, &OSPIM_Cfg_Struct, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI1_Init 2 */

  /* USER CODE END OCTOSPI1_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  hrng.Init.ClockErrorDetection = RNG_CED_ENABLE;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
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
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  huart2.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_USB_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin|ARD_D4_Pin|ARD_D7_Pin|GPIO_PIN_5|GPIO_PIN_14|SPBTLE_RF_RST_Pin
                          |ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin|ISM43362_BOOT0_Pin|LED2_Pin|SPSGRF_915_SDN_Pin
                          |ARD_D5_Pin|SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|PMOD_SPI2_SCK_Pin|STSAFE_A110_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ST25DV04K_RF_DISABLE_Pin */
  GPIO_InitStruct.Pin = ST25DV04K_RF_DISABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ST25DV04K_RF_DISABLE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin ST25DV04K_GPO_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin|ST25DV04K_GPO_Pin|SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_EXTI13_Pin VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin|VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D10_Pin ARD_D4_Pin ARD_D7_Pin SPBTLE_RF_RST_Pin
                           ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D10_Pin|ARD_D4_Pin|ARD_D7_Pin|GPIO_PIN_5|GPIO_PIN_14|SPBTLE_RF_RST_Pin
                          |ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  /*GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);*/

  /*Configure GPIO pin : ARD_D3_Pin */
  GPIO_InitStruct.Pin = ARD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D6_Pin */
  GPIO_InitStruct.Pin = ARD_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(ARD_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D8_Pin ISM43362_BOOT0_Pin LED2_Pin SPSGRF_915_SDN_Pin
                           ARD_D5_Pin SPSGRF_915_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = ARD_D8_Pin|ISM43362_BOOT0_Pin|LED2_Pin|SPSGRF_915_SDN_Pin
                          |ARD_D5_Pin|SPSGRF_915_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI10_Pin LSM6DSL_INT1_EXTI11_Pin USB_OTG_FS_PWR_EN_Pin ARD_D2_Pin
                           HTS221_DRDY_EXTI15_Pin PMOD_IRQ_EXTI2_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI10_Pin|LSM6DSL_INT1_EXTI11_Pin|USB_OTG_FS_PWR_EN_Pin|ARD_D2_Pin
                          |HTS221_DRDY_EXTI15_Pin|PMOD_IRQ_EXTI2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin PMOD_SPI2_SCK_Pin STSAFE_A110_RESET_Pin */
  GPIO_InitStruct.Pin = SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|PMOD_SPI2_SCK_Pin|STSAFE_A110_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OTG_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_ID_Pin USB_OTG_FS_DM_Pin USB_OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_ID_Pin|USB_OTG_FS_DM_Pin|USB_OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : INTERNAL_SPI3_SCK_Pin INTERNAL_SPI3_MISO_Pin INTERNAL_SPI3_MOSI_Pin */
  GPIO_InitStruct.Pin = INTERNAL_SPI3_SCK_Pin|INTERNAL_SPI3_MISO_Pin|INTERNAL_SPI3_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/**
  * @brief  This function process the message received by the TCP server.
  * @retval None
  */
void processReceivedData(uint8_t* data, int dataLength) {
    MessageType messageType = (MessageType)data[0]; // First byte is the message type

    switch (messageType) {
        case MESSAGE_TYPE_HELLO: {
            if (dataLength > 1) {
                HelloMessage hello;
                hello.type = messageType;
                strncpy(hello.message, (char*)data + 1, sizeof(hello.message) - 1);
                hello.message[sizeof(hello.message) - 1] = '\0';  // Ensure null termination
                printf("Received message: %s\n\r", hello.message);
            }
            break;
        }
        case MESSAGE_TYPE_SENSORS_REQUEST: {
          sendData(MESSAGE_TYPE_UPDATE);
          printf("Sending data as desired\n\r");
          break;
        }
        case MESSAGE_TYPE_TIMER_UPDATE: {
            if (dataLength == BUFFER_SIZE_TIMER_UPDATE) {  // 1 (type) + 4 (uint32_t timer)
                TimerUpdateMessage timerUpdate;
                timerUpdate.type = messageType;
                memcpy(&timerUpdate.timer_value, data + 1, sizeof(timerUpdate.timer_value));
                // Optionally convert from network to host byte order (if needed)
                //timerUpdate.timer_value = ntohl(timerUpdate.timer_value);
                printf("Received Timer Update: %u seconds\n\r", timerUpdate.timer_value);
                // Update the internal timer with timerUpdate.timer_value
            }
            break;
        }
        case MESSAGE_TYPE_THRESHOLD_UPDATE: {
            if (dataLength == BUFFER_SIZE_THRESHOLD_UPDATE) { // 1 (type) + 4 * 3 (float thresholds)
                ThresholdUpdateMessage thresholdUpdate;
                thresholdUpdate.type = messageType;
                memcpy(&thresholdUpdate.temperature_threshold, data + 1, sizeof(thresholdUpdate.temperature_threshold));
                memcpy(&thresholdUpdate.humidity_threshold, data + 5, sizeof(thresholdUpdate.humidity_threshold));
                memcpy(&thresholdUpdate.light_threshold, data + 9, sizeof(thresholdUpdate.light_threshold));

                printf("Received Threshold Update: Temp=%.2f, Hum=%.2f, Light=%.2f\n\r",
                       thresholdUpdate.temperature_threshold, thresholdUpdate.humidity_threshold,
                       thresholdUpdate.light_threshold);
                // Check if the internal thresholds are within the limits TODO
                // Update the internal thresholds with these values
                gTempThreshold = thresholdUpdate.temperature_threshold;
                gHumThreshold = thresholdUpdate.humidity_threshold;
                gLightThreshold = thresholdUpdate.light_threshold;
                printf("Threshold updated.\n\r");
                sendData(MESSAGE_TYPE_UPDATE);
            }
            break;
        }
        default:
            printf("Unknown message type received: %d\n", messageType);
            break;
    }
}

/**
  * @brief  This function parses the string and initilize the message structure
  * @retval None
  */
messageReceived parseMessage(const char *rxData) {
    messageReceived message;
    char *token;
    char *str_copy; // Create a copy of the string, strtok modifies it

    // Initialize to safe default values in case parsing fails
    message.type  = 0;
    message.time  = 0;
    message.temp  = DEFAULT_TEMP;
    message.hum   = DEFAULT_HUM;
    message.light = DEFAULT_LIGHT;

    // Create a copy of the input string since strtok modifies the string
    str_copy = strdup(rxData);  // IMPORTANT:  Allocate memory for the copy

    if (str_copy == NULL) {
        printf("Error: Memory allocation failed for string copy.\n");
        return message; // Return default initialized struct
    }

    // Parse the type
    token = strtok(str_copy, ","); // Tokenize by comma
    if (token != NULL) {
        message.type = (uint8_t)atoi(token); // Convert string to integer
        if (message.type > 2) {
            printf("Error: Invalid type value: %d\n", message.type);
            message.type = 0; // Reset to default
        }
    } else {
        printf("Error: Missing type value\n");
        goto cleanup; // Jump to cleanup to free memory
    }

    // Handle different types
    if (message.type == 0) {
        // Use default values for all other fields (already initialized)
        printf("Type 0: Using default values.\n");
    } else if (message.type == 1) {
        // Parse the time
        token = strtok(NULL, ","); // Get the next token
        if (token != NULL) {
            message.time = (uint8_t)atoi(token);
            if (message.time > 3) {
                printf("Error: Invalid time value: %d\n", message.time);
                message.time = 0;  //Reset to Default
            }
        } else {
            printf("Error: Missing time value for type 1\n");
        }

        // Use default values for temp, hum, light (already initialized)
        printf("Type 1: Using default values for temp, hum, light.\n");
    } else if (message.type == 2) {
        // Parse temp, hum, and light
        token = strtok(NULL, ","); // Get temp
        if (token != NULL) {
            message.temp = (float)atof(token);
        } else {
            printf("Error: Missing temp value for type 2\n");
        }

        token = strtok(NULL, ","); // Get hum
        if (token != NULL) {
            message.hum = (float)atof(token);
        } else {
            printf("Error: Missing hum value for type 2\n");
        }

        token = strtok(NULL, ","); // Get light
        if (token != NULL) {
            message.light = (float)atof(token);
        } else {
            printf("Error: Missing light value for type 2\n");
        }

        printf("Type 2: Using parsed values.\n");
    }

cleanup:
    free(str_copy); // Free the allocated memory
    return message;
}

/**
  * @brief  This function creates the message structure and calls sendMessage
  * @retval None
  */
void sendData(int type) {
    mesageSent myMessage;
    myMessage.type = type;
    myMessage.temp = gTemperature;
    myMessage.hum = gHumidity;
    myMessage.light = gLight;
    myMessage.temperature_threshold = gTempThreshold;
    myMessage.humidity_threshold = gHumThreshold;
    myMessage.light_threshold = gLightThreshold;

    WIFI_Status_t sendStatus = sendMessage(&myMessage);

    if (sendStatus == WIFI_STATUS_OK) {
        printf("Message sent successfully!\n\r");
    } else {
        printf("Error sending message!\n\r");
    }
}

/**
  * @brief  This function packs the message
  * @retval uint8_t*
  */
uint8_t* packMessage(const mesageSent* message, uint32_t* packedSize) {
    // Calculate the size of the packed data
    uint32_t size = sizeof(message->type) +
                   sizeof(message->temp) +
                   sizeof(message->hum) +
                   sizeof(message->light) +
                   sizeof(message->temperature_threshold) +
                   sizeof(message->humidity_threshold) +
                   sizeof(message->light_threshold);


//    printf("Tamano bueno bueno\n\r",size);
//    printf("The value of my_uint32 is: %x\n\r", size);
    // Allocate memory for the packed data
    uint8_t* packedData = (uint8_t*)malloc(size);

    if (packedData == NULL) {
        printf("Error: Memory allocation failed for packed data.\n\r");
        *packedSize = 0;
        return NULL;
    }

    // Pack the data into the byte array
    uint8_t* ptr = packedData;

    memcpy(ptr, &message->type, sizeof(message->type));
    ptr += sizeof(message->type);

    memcpy(ptr, &message->temp, sizeof(message->temp));
    ptr += sizeof(message->temp);

    memcpy(ptr, &message->hum, sizeof(message->hum));
    ptr += sizeof(message->hum);

    memcpy(ptr, &message->light, sizeof(message->light));
    ptr += sizeof(message->light);

    memcpy(ptr, &message->temperature_threshold, sizeof(message->temperature_threshold));
    ptr += sizeof(message->temperature_threshold);

    memcpy(ptr, &message->humidity_threshold, sizeof(message->humidity_threshold));
    ptr += sizeof(message->humidity_threshold);

    memcpy(ptr, &message->light_threshold, sizeof(message->light_threshold));
    ptr += sizeof(message->light_threshold);

    *packedSize = size;
    printf("Packed Data (Hex): ");
    for (uint32_t i = 0; i < size; i++) {
      printf("%02X ", packedData[i]); // %02X formats the byte as a two-digit hexadecimal number with leading zero if needed.
    }
//	printf("\n\r");
//	printf("YA ESTAMOS AQUIII");
    return packedData;
}

/**
  * @brief  This function sends the message
  * @retval WIFI_Status_t
  */
WIFI_Status_t sendMessage(const mesageSent* message) {
    uint32_t packedSize;
    uint8_t* packedData = packMessage(message, &packedSize);
    if (packedData == NULL) {
        printf("Error: packedData es NULL\n");
        return 1;
    }

    if (packedData == NULL || packedSize == 0) {
        // Handle memory allocation failure
        return WIFI_STATUS_ERROR;
    }

    printf("(%ld bytes)\n\r", packedSize); //Print the real Packed Size

    // Assuming WIFI_SendData takes a byte array and its length
    WIFI_Status_t message_status = WIFI_SendData(0, packedData, packedSize, NULL, 5000);

    // Free the allocated memory
    free(packedData);

    return message_status;
}

/**
  * @brief  This function reads the values from the Light sensor
  * @retval uint16_t
  */
uint16_t readLightLevelValues(){
  uint16_t light_value;
  HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1,100) == HAL_OK)
    {
      light_value = HAL_ADC_GetValue(&hadc1);
      light_value = (light_value / 4095.0) * 100.0; /* adc= 12 bits, normalize and then goes from 0 to 100*/
      HAL_ADC_Stop(&hadc1);
    }
    return light_value;
}


/**
  * @brief  This function reads the values from the Temperature and Humidity sensor
  * @retval None
  */
void readSensorValues(){
  float temp_value;
  float hum_value;
  uint16_t light_value;

  temp_value = BSP_TSENSOR_ReadTemp();    // Read Temperature
  hum_value = BSP_HSENSOR_ReadHumidity(); // Read Humidity
  light_value = readLightLevelValues();   // Read Light Level

  printf("Read Sensor values: Temp=%.2f, Hum=%.2f & Light=%d\n\r",temp_value, hum_value, light_value);

  gTemperature = temp_value;
  gHumidity = hum_value;
  gLight = light_value;
}

/**
  * @brief  This function checks the thresholds and sends the corresponding mesage
  * @retval None
  */
void checkThresholds(){
  if(gTemperature > gTempThreshold){
    sendData(MESSAGE_TYPE_ALARM);
    gTempAlarm = true;
  }else if (gTempAlarm == true){
    sendData(MESSAGE_TYPE_UPDATE);
    gTempAlarm = false;
  }

  if(gHumidity > gHumThreshold){
    sendData(MESSAGE_TYPE_ALARM);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // Encender el LED
    gHumAlarm = true;
  }else if (gHumAlarm == true){
    sendData(MESSAGE_TYPE_UPDATE);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // Encender el LED
    gHumAlarm = false;
  }

  if(gLight > gLightThreshold){
    sendData(MESSAGE_TYPE_ALARM);
    gLightAlarm = true;
  }else if(gLightAlarm == true){
    sendData(MESSAGE_TYPE_UPDATE);
    gLightAlarm = false;
  }
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

// SPI3 Interruption request handler for the Wifi module (send/receive AT cmd via SPI)
void SPI3_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi);
}


PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART2 and Loop until the end of transmission */
  while (HAL_OK != HAL_UART_Transmit(&huart1, (uint8_t *) &ch, 1, 30000))
  {
    ;
  }
  return ch;
}
GETCHAR_PROTOTYPE
{
  /* Place your implementation of fgetc here */
  /* e.g. read a character on USART and loop until the end of read */
  uint8_t ch = 0;
  while (HAL_OK != HAL_UART_Receive(&huart1, (uint8_t *)&ch, 1, 30000))
  {
    ;
  }
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
