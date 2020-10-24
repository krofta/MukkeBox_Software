/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "TDA7719.h"
#include "ssd1306.h"
//#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DISPLAY 1
#define ADC_BUF_SIZE 6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static const uint8_t OLED_ADDR = 0x3C;	// SSD1306
static const uint8_t AUDIO_DSP = 0x44;	// TDA7719



uint16_t ADC_buffer[ADC_BUF_SIZE];
uint8_t adc_flag = 0;

uint8_t timer_flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

GPIO_PinState state_3v3 = GPIO_PIN_SET;
GPIO_PinState charger_plugged = GPIO_PIN_SET;
const int GAIN = 4;
const int BASS = 3;
const int MIDDLE = 2;
const int TREBLE = 1;
const int PACK = 5;
const int GAIN_STEPS = 4096/30;
const int GAIN_HYST = 30;
// Faktor aus Debug Wert, und Messung mit Multimeter bestimmt; Stimmt auf 10mV mit Fluke Multimeter überein
const double PACK_FAKTOR = 14.86 / 1698.0;
const double PACK_LOW = 14.0;
const double PACK_FULL = 16.6;
const double PACK_HYST = 0.1;
static int pack_sense = 0;
static int pack_val = 0;
static double pack_voltage_buf[5] = {0,0,0,0,0};
static int buf_index = 0;
static double pack_voltage = 0.0;
static int pack_low = 1;
static int pack_full = 0;
static int power_rise = 0;
static int dsp_delay = 0;
static int FAIL = 0;

static int old_gain=0, old_bass = 0, old_middle=0, old_treble=0;
static int gain = 0, bass = 0, middle = 0, treble = 0;

static int splash_sceen = 0;

static int write_bat = 0;

void write_screen(int bass, int middle, int treble, double battery){

#if DISPLAY == 1
	char s[7];
	int vorkomma = 0, nachkomma = 0;
	ssd1306_Fill(Black);
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString("GurkiBox   V03", Font_6x8, White);
	//bass
	if(state_3v3 == GPIO_PIN_RESET){
		ssd1306_SetCursor(0, 10);
		ssd1306_WriteString("Bass:", Font_6x8, White);
		ssd1306_SetCursor(64, 10);
		itoa((int16_t)(bass-15), s, 10);
		if(bass>15)ssd1306_WriteString("+", Font_6x8, White);
		ssd1306_WriteString(s, Font_6x8, White);
		ssd1306_WriteString("bB", Font_6x8, White);
		//Middle
		ssd1306_SetCursor(0, 20);
		ssd1306_WriteString("Middle:", Font_6x8, White);
		ssd1306_SetCursor(64, 20);
		itoa((int16_t)(middle-15), s, 10);
		if(middle>15)ssd1306_WriteString("+", Font_6x8, White);
		ssd1306_WriteString(s, Font_6x8, White);
		ssd1306_WriteString("bB", Font_6x8, White);
		// Treble
		ssd1306_SetCursor(0, 30);
		ssd1306_WriteString("Treble:", Font_6x8, White);
		ssd1306_SetCursor(64, 30);
		itoa((int16_t)(treble-15), s, 10);
		if(treble>15)ssd1306_WriteString("+", Font_6x8, White);
		ssd1306_WriteString(s, Font_6x8, White);
		ssd1306_WriteString("bB", Font_6x8, White);
	}
	// Battery
	vorkomma=(int)battery;
	nachkomma = (int)((battery-((double)vorkomma))*100);
	ssd1306_SetCursor(0, 40);
	ssd1306_WriteString("Akku:", Font_6x8, White);
	ssd1306_SetCursor(64, 40);
	itoa((int16_t)vorkomma, s, 10);
	ssd1306_WriteString(s, Font_6x8, White);
	ssd1306_WriteString(".", Font_6x8, White);
	itoa((int16_t)nachkomma, s, 10);
	ssd1306_WriteString(s, Font_6x8, White);
	ssd1306_WriteString("V", Font_6x8, White);

	if(charger_plugged == GPIO_PIN_RESET){
		uint8_t x = 110, y = 0;
		//Linker Ramen Akku
		ssd1306_Line(x,y,x,y+6, White);
		ssd1306_Line(x,y,x+3,y, White);
		ssd1306_Line(x,y+6,x+3,y+6, White);

		//Rechter Ramen Akku
		ssd1306_Line(x+9,y,x+11,y, White);
		ssd1306_Line(x+9,y+6,x+11,y+6, White);
		ssd1306_Line(x+11,y,x+11,y+6, White);
		ssd1306_Line(x+12,y+2,x+12,y+4, White);
		//Strom Symbol
		if(pack_full == 0){
			ssd1306_Line(x+6,y,x+4,y+2, White);
			ssd1306_Line(x+7,y+4,x+5,y+6, White);
			ssd1306_Line(x+4,y+3,x+7,y+3, White);
			ssd1306_DrawPixel(x+5,y+2,White);
			ssd1306_DrawPixel(x+6,y+4,White);
		}
		else{
			ssd1306_Line(x+2,y,x+9,y, White);
			ssd1306_Line(x+2,y+1,x+9,y+1, White);
			ssd1306_Line(x+2,y+2,x+9,y+2, White);
			ssd1306_Line(x+2,y+3,x+9,y+3, White);
			ssd1306_Line(x+2,y+4,x+9,y+4, White);
			ssd1306_Line(x+2,y+5,x+9,y+5, White);
			ssd1306_Line(x+2,y+6,x+9,y+6, White);
		}
	}

	ssd1306_UpdateScreen();
#endif
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
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  if(HAL_OK != HAL_ADCEx_Calibration_Start(&hadc))
	  Error_Handler();

  if(HAL_ADC_Start_DMA(&hadc, &ADC_buffer, ADC_BUF_SIZE) != HAL_OK)
	  Error_Handler();

  if(HAL_OK != HAL_TIM_Base_Start(&htim1))
	  Error_Handler();
  if(HAL_OK != HAL_TIM_Base_Start(&htim3))
	  Error_Handler();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // Endstufe initial deaktiviert
  HAL_GPIO_WritePin(MUTE_AMP1_GPIO_Port,MUTE_AMP1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(STBY_AMP1_GPIO_Port,STBY_AMP1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MUTE_AMP2_GPIO_Port,MUTE_AMP2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(STBY_AMP2_GPIO_Port,STBY_AMP2_Pin, GPIO_PIN_RESET);
  //HAL_GPIO_WritePin(STEPUP_DISABLE_GPIO_Port, STEPUP_DISABLE_Pin, GPIO_PIN_SET);

  GPIO_PinState state;
  uint8_t test = 0;
  uint8_t adresses[5] = {0,0,0,0,0};


// I2C Scan
	HAL_StatusTypeDef result;
	int i = 0;
	for (i=1; i<128; i++)
	{

	   //the HAL wants a left aligned i2c address
	   //&hi2c1 is the handle
	   //(uint16_t)(i<<1) is the i2c address left aligned
	   //retries 2
	   //timeout 2

	  result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 2, 2);
	  if (result != HAL_OK) // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
	  {
		  ;
	  }
	  if (result == HAL_OK)
	  {
		  adresses[test]= i;
		  test++;
	  }
	}

#if DISPLAY == 1
  	  ssd1306_Init();
  	  ssd1306_Fill(Black);
  	  ssd1306_SetCursor(0, 20);
  	  ssd1306_WriteString("Moin Digga!", Font_6x8, White);
  	  ssd1306_UpdateScreen();
#endif




  while (1)
  {
	  // Alles abschalten, wenn eine Endstufe in Störung geht. Fehler am Display anzeigen
	  if((HAL_GPIO_ReadPin(DIAG_AMP1_GPIO_Port, DIAG_AMP1_Pin) == GPIO_PIN_RESET) ||
			  (HAL_GPIO_ReadPin(DIAG_AMP2_GPIO_Port, DIAG_AMP2_Pin) == GPIO_PIN_RESET) ||
			  (HAL_GPIO_ReadPin(BT_FAULT_GPIO_Port, BT_FAULT_Pin) == GPIO_PIN_RESET)){
		  // Endstufen abschalten
		  HAL_GPIO_WritePin(MUTE_AMP1_GPIO_Port,MUTE_AMP1_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(STBY_AMP1_GPIO_Port,STBY_AMP1_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MUTE_AMP2_GPIO_Port,MUTE_AMP2_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(STBY_AMP2_GPIO_Port,STBY_AMP2_Pin, GPIO_PIN_RESET);
		  // Power abschalten
		  HAL_GPIO_WritePin(POWER_GPIO_Port,POWER_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(ENABLE_9V_GPIO_Port,ENABLE_9V_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(BT_EN_GPIO_Port,BT_EN_Pin, GPIO_PIN_RESET);
#if DISPLAY == 1
			char s[7];
			int vorkomma = 0, nachkomma = 0;
			ssd1306_Fill(Black);
			ssd1306_SetCursor(0, 0);
			ssd1306_WriteString("GurkiBox   V03", Font_6x8, White);
			//bass
			if((HAL_GPIO_ReadPin(DIAG_AMP1_GPIO_Port, DIAG_AMP1_Pin) == GPIO_PIN_RESET)){
				ssd1306_SetCursor(0, 10);
				ssd1306_WriteString("Error Amp1", Font_6x8, White);
			}
			if((HAL_GPIO_ReadPin(DIAG_AMP2_GPIO_Port, DIAG_AMP2_Pin) == GPIO_PIN_RESET)){
				ssd1306_SetCursor(0, 10);
				ssd1306_WriteString("Error Amp2", Font_6x8, White);
			}
			if((HAL_GPIO_ReadPin(BT_FAULT_GPIO_Port, BT_FAULT_Pin) == GPIO_PIN_RESET)){
				ssd1306_SetCursor(0, 10);
				ssd1306_WriteString("Error ", Font_6x8, White);
				ssd1306_SetCursor(0, 20);
				ssd1306_WriteString("Bluetooth ", Font_6x8, White);
			}
			ssd1306_UpdateScreen();
#endif
		  while(1){
			  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
			  ;
		  }
	  }


	  if(adc_flag == 1){
		  // Run LED
		  HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);

		  if(HAL_ADC_Start_DMA(&hadc, &ADC_buffer, ADC_BUF_SIZE) != HAL_OK)
			  Error_Handler();

		  if(HAL_OK != HAL_TIM_Base_Start(&htim1))
			  Error_Handler();
		  adc_flag = 0;
		  //TDA7719_begin(hi2c1);

		  // Überprüfen, ob Ladeadapter eingesteckt
		  charger_plugged = HAL_GPIO_ReadPin(CHARGER_PLUGGED_GPIO_Port, CHARGER_PLUGGED_Pin);

		  // Akku Spannung überwachen
		  if(HAL_GPIO_ReadPin(EN_PACK_SENSE_GPIO_Port,EN_PACK_SENSE_Pin) == GPIO_PIN_SET){
			  //1698 ~ 14,76V
			  pack_val = ADC_buffer[PACK];
			  // Ringpuffer
			  pack_voltage_buf[buf_index] = ((double)pack_val) * PACK_FAKTOR;
			  double rms = 0;
			  for(int i = 0; i<5; i++)rms+=pack_voltage_buf[i];
			  rms/=5.0;
			  pack_voltage = rms;
			  buf_index = buf_index == 4 ? 0 : buf_index +1;
		  }
		  pack_sense++;
		  HAL_GPIO_WritePin(EN_PACK_SENSE_GPIO_Port,EN_PACK_SENSE_Pin , ((pack_sense%10) == 0));
		  write_bat++;
#if DISPLAY == 1
		  if((write_bat % 1000)  == 0)
			  write_screen(bass, middle, treble, pack_voltage);
#endif


		  // Charger abschalten, wenn Spannung überschritten wird
		  pack_full = pack_voltage > (PACK_FULL + (PACK_HYST*2)) ? 1 : pack_voltage < (PACK_FULL - (PACK_HYST*2)) ? 0 : pack_full;
		  // TEST pack_full = pack_voltage > (15.0 + (PACK_HYST*2)) ? 1 : pack_voltage < 15.0 ? 0 : 0;
		  HAL_GPIO_WritePin(CHARGER_DISABLE_GPIO_Port,CHARGER_DISABLE_Pin , pack_full);

		  // Versorgungsspannung schalten
		  pack_low = (pack_voltage > (PACK_LOW + PACK_HYST)) ? 0 : (pack_voltage < PACK_LOW) ? 1 : pack_low;

		  state_3v3 = HAL_GPIO_ReadPin(U3V3_ENABLED_GPIO_Port, U3V3_ENABLED_Pin);
		  int power_on = (state_3v3 == GPIO_PIN_RESET) && (pack_low == 0);
		  HAL_GPIO_WritePin(POWER_GPIO_Port,POWER_Pin, power_on);

		  // Versorgungsspannung für Audio-DSP einschalten
		  HAL_GPIO_WritePin(ENABLE_9V_GPIO_Port,ENABLE_9V_Pin, power_on);
		  // Bluetooth Modul einschalten
		  HAL_GPIO_WritePin(BT_EN_GPIO_Port,BT_EN_Pin, power_on);


		  // DSP initialisieren
		  if(power_rise < HAL_GPIO_ReadPin(POWER_GPIO_Port,POWER_Pin)){
#if DISPLAY == 1
			  write_screen(bass, middle, treble, pack_voltage);
#endif
			  dsp_delay++;
		  }
		  dsp_delay = dsp_delay == 0 ? 0 : dsp_delay < 11 ? dsp_delay + 1 : dsp_delay;
		  if( dsp_delay == 10){
			  // 1s gewartet nach power up
			  // Endstufen einschalten
			  HAL_GPIO_WritePin(MUTE_AMP1_GPIO_Port,MUTE_AMP1_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(STBY_AMP1_GPIO_Port,STBY_AMP1_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(MUTE_AMP2_GPIO_Port,MUTE_AMP2_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(STBY_AMP2_GPIO_Port,STBY_AMP2_Pin, GPIO_PIN_SET);
			  TDA7719_begin(hi2c1);
			  TDA7719_volume((int8_t)gain);
			  TDA7719_bass((int8_t)bass);
			  TDA7719_middle((int8_t)middle);
			  TDA7719_treble((int8_t)treble);

		  }
		  power_rise = HAL_GPIO_ReadPin(POWER_GPIO_Port,POWER_Pin);
		  if(power_rise == 0) dsp_delay = 0;

		  // Splash screen -  Moin Digga!
		  splash_sceen=splash_sceen <= 100 ? splash_sceen + 1 : splash_sceen;
		  if(splash_sceen== 100){
#if DISPLAY == 1
			  write_screen(bass, middle, treble, pack_voltage);
#endif
		  }

		  gain = ADC_buffer[GAIN]/GAIN_STEPS;
		  if(old_gain != gain && ((ADC_buffer[GAIN]%GAIN_STEPS) > GAIN_HYST) && (splash_sceen > 100)){
			  old_gain = gain;
			  TDA7719_volume((int8_t)gain);
#if DISPLAY == 1
			  write_screen(bass, middle, treble, pack_voltage);
#endif
		  }

		  bass = ADC_buffer[BASS]/GAIN_STEPS;
		  if(old_bass != bass && ((ADC_buffer[BASS]%GAIN_STEPS) > GAIN_HYST) && (splash_sceen > 100)){
			  old_bass = bass;
			  TDA7719_bass((int8_t)bass);
#if DISPLAY == 1
			  write_screen(bass, middle, treble, pack_voltage);
#endif
		  }

		  middle = ADC_buffer[MIDDLE]/GAIN_STEPS;
		  if(old_middle != middle && ((ADC_buffer[MIDDLE]%GAIN_STEPS) > GAIN_HYST) && (splash_sceen > 100)){
			  old_middle = middle;
			  TDA7719_middle((int8_t)middle);
#if DISPLAY == 1
			  write_screen(bass, middle, treble, pack_voltage);
#endif
		  }

		  treble = ADC_buffer[TREBLE]/GAIN_STEPS;
		  if(old_treble != treble && ((ADC_buffer[TREBLE]%GAIN_STEPS) > GAIN_HYST) && (splash_sceen > 100)){
			  old_treble = treble;
			  TDA7719_treble((int8_t)treble);
#if DISPLAY == 1
			  write_screen(bass, middle, treble, pack_voltage);
#endif
		  }

	  }

	  // Power Schalter lesen zum aktiven ausschalten
	  state = HAL_GPIO_ReadPin(POWER_GPIO_Port, POWER_Pin);
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_6;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hi2c1.Init.Timing = 0x00103D84;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim3.Init.Prescaler = 7999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, EN_PACK_SENSE_Pin|CHARGER_DISABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MUTE_AMP2_Pin|STEPUP_DISABLE_Pin|STBY_AMP2_Pin|POWER_Pin 
                          |RST_BT_Pin|MUTE_AMP1_Pin|STBY_AMP1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, STATUS_LED_Pin|ENABLE_9V_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BT_EN_GPIO_Port, BT_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EN_PACK_SENSE_Pin CHARGER_DISABLE_Pin */
  GPIO_InitStruct.Pin = EN_PACK_SENSE_Pin|CHARGER_DISABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : CHARGER_PLUGGED_Pin */
  GPIO_InitStruct.Pin = CHARGER_PLUGGED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CHARGER_PLUGGED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO1_Pin GPIO2_Pin GPIO3_Pin GPIO4_Pin 
                           DIAG_AMP2_Pin U3V3_ENABLED_Pin DIAG_AMP1_Pin */
  GPIO_InitStruct.Pin = GPIO1_Pin|GPIO2_Pin|GPIO3_Pin|GPIO4_Pin 
                          |DIAG_AMP2_Pin|U3V3_ENABLED_Pin|DIAG_AMP1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MUTE_AMP2_Pin STEPUP_DISABLE_Pin STBY_AMP2_Pin POWER_Pin 
                           RST_BT_Pin MUTE_AMP1_Pin STBY_AMP1_Pin */
  GPIO_InitStruct.Pin = MUTE_AMP2_Pin|STEPUP_DISABLE_Pin|STBY_AMP2_Pin|POWER_Pin 
                          |RST_BT_Pin|MUTE_AMP1_Pin|STBY_AMP1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STATUS_LED_Pin ENABLE_9V_Pin */
  GPIO_InitStruct.Pin = STATUS_LED_Pin|ENABLE_9V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CHARGER_STAT1_Pin BT_FAULT_Pin */
  GPIO_InitStruct.Pin = CHARGER_STAT1_Pin|BT_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CHARGER_STAT2_Pin */
  GPIO_InitStruct.Pin = CHARGER_STAT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CHARGER_STAT2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BT_EN_Pin */
  GPIO_InitStruct.Pin = BT_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BT_EN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){


  if(HAL_OK != HAL_TIM_Base_Start(&htim1))
	  Error_Handler();
	adc_flag = 1;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
		timer_flag = 1;
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler()
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
