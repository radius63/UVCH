/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention 
  * Программа управления работой RF-платы блока УВЧ
  * К микроконтроллеру подключены:
  * UART1(115200) - через конвертер USB/UART получает команды на конфигурирование (3 байта)
  * UART2(921600) - усилитель мощности (команда - 1 байт, ответ - 1байт)                        
  * SPI1 (master) - модулятор, демодулятор, два сдвиговых регистра
	* SPI2 (slave) - получение команд от ARMа
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Nastroyki.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

struct UART_DATA uart1;
struct tpSettings setings;
// ==============================================================================
// FLASH Settings 
// ==============================================================================
#define MY_FLASH_PAGE_ADDR 0x800FC00    // начальный адрес последней страницы FLASH
#define SETTINGS_WORDS sizeof(setings)/4
#define APPLICATION_ADDRESS 0x08003000  //адрес начала программы
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
volatile uint16_t adc[6] = {0};			// данные с АЦП
volatile uint8_t RX1[8] = {0}; 			// буфер приема UART1
volatile uint8_t RX2[4] = {0}; 			// буфер приема UART2 
volatile uint8_t TX1[64];						// буфер передачи UART1
volatile uint8_t TX_SPI1[3] = {0}; 	// буфер передачи по SPI1
volatile uint8_t RX_SPI1[2] = {0}; 	// буфер приема с SPI1
volatile uint8_t TX_SPI2[4] = {0}; 	// буфер передачи по SPI2
volatile uint8_t RX_SPI2[4] = {0}; 	// буфер приема с SPI2
const uint8_t versionPO_1 = 6;				// версия ПО целая часть
const uint8_t versionPO_2 = 0;				// версия ПО дробная часть
uint8_t Byte1, Byte2; 	// для UART1, UART2 
int mode = 0; 					// 1 - рабочий режим, 0 - настроечный режим
int flagConfig = 0; 		// 1 - сконфигурировать пины на вход, 	0 - не нужно конфигурировать
int flagWork = 0; 			// 1 - сконфигурировать пины на выход, 	0 - не нужно конфигурировать
int flagADC = 0;				// 1 - преобразование завершено
int flagPwrAmpl = 0;		// 1 - есть ответ от усилителя мощности
int flagSleepPA = 1;    // 1 - усилитель в режиме Sleep
int flagSpi2 = 0;				// флаг приема/отправки данных по SPI2
int flagBB_OK = 0;			// эквивалент сигнала BB_OK
int flag1S = 0;					// флаг выставляется 1 раз в секунду
int flag500Hz_En = 0; 	// флаг наличия частоты 500 Гц
int flagFrqRX = 0; 			// флаг изменения частоты приемника
int flagFrqTX = 0; 			// флаг изменения частоты передатчика
int AntSw = 0;					// состояние антенного переключателя
int numrx = 0;					// номер принятой посылки по SPI2
int numtx = 0; 					// номер отправленной посылки по SPI2
uint16_t rssi = 0; 			// ADC_IN1 rssi
uint16_t power1 = 0; 		// ADC_IN4 выходная мощность с 1-й антенны
uint16_t power2 = 0; 		// ADC_IN5 выходная мощность со 2-й антенны
uint16_t ksvn1 = 0; 		// ADC_IN6 КСВН 1-й антенны
uint16_t ksvn2 = 0; 		// ADC_IN8 КСВН 2-й антенны
uint16_t status = 0x0000;		// статус (состояние аппаратуры)
/*
 0: =1, отказ RF, 
 1: =1, перегрев RF, 
 2: =1, отказ УМ, 
 3: =1, перегрев УМ, 
 4: =1, отказ измерителя КСВН, 
 5: =1, перегрев измерителя КСВН, 
 6: =1, ошибка антенны (ANT_FAIL), 
 7: =?, КЗ или ХХ (ANT_SC),
 8: =1, КСВН превышает порог, 
 9: =1, Мощность превышает порог
*/
//int flagAntLna = 0;  // флаг антенного МШУ
uint16_t AntLna = 16;
uint16_t frqTX;							// частота передатчика
uint16_t frqRX;							// частота приемника
uint8_t frqLpf;						  // частота среза ФНЧ
float tempr = 0;			// Temperature Sensor
uint16_t usKodNumChanOld = 0xFF, usKodNumChanNew = 0; // код номера канала
uint32_t T2 = 0; // счетчик для таймера T2 срабатывает каждые 100 мкс.
uint32_t T3 = 0; // счетчик для таймера T3 срабатывает каждые 10 мкс.
uint32_t T1S = 0; // счетчик для отсчета 1 с 
uint16_t buf_tx = 0;
uint16_t buf_rx = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void FLASH_Init(void);
void FLASH_ReadSettings(void);						// чтение настроек из FLASH
void FLASH_WriteSettings(void);						// запись настроек во FLASH
void Terminal(void);											// прием и вывод в терминал
void prntVar(void); 											// вывести все значения из FLASH
void saveVar(void);												// сохранить все значения во FLASH
int  setFreqChannl(uint16_t uiNum); 			// установить частотный канал
void setFreqTX(uint16_t uiNum);						// установить частоту передатчика
void setFreqRX(uint16_t uiNum);						// установить частоту приемника
void setLPF(char FrLPF);									// установить полосу ФНЧ
void setOperMode(char mode);							// установить режим работы
void setPot(char pot, char data);					// установить значение потенциометров
void setShiftReg(char reg, char data);		// записать сдвиговые регистры
void setFiltr(char filtr, uint8_t data);	// загрузка фильтров
void setModul(char reg, char data);				// загрузка регистров модулятора
uint8_t readModul(char reg);							// чтение регистров модулятора
uint8_t setDemod(char reg, char data);		// загрузка регистров демодулятора 
uint8_t readDemod(char reg);							// чтение регистров демодулятора
void wrtVar(char addr, char data);				// ввод значений без записи во FLASH
void readVar(char addr);									// чтение значения из ОЗУ
void setAttTX(char data);									// установка аттенюаторов передатчика
void setAttLNA(char data);								// установка аттенюаторов МШУ
void telemetr(char data); 								// выдача телеметрии через UART1
void telemetrSPI(char data); 							// выдача телеметрии через SPI
void pinOutput(void);											// переключение пинов на выход
void pinInput(void);											// переключение пинов на вход
void startADC(void);											// запуск преобразования в АЦП
void DelayTic(uint32_t Tic);							// задержка в тиках контроллера
void ComndProcessing(void);								// обработка команд от SPI2
	
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
	__disable_irq(); //запрещаем прерывания
	SCB->VTOR = 0x08003000; //переносим начало вектора прерываний по указанному адресу
	__enable_irq();
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
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	FLASH_Init();
	FLASH_ReadSettings(); // считываем из FLASH значения переменных
	HAL_UART_Receive_IT(&huart1, &Byte1, 1); // заказываем байт по UART1
	HAL_UART_Receive_IT(&huart2, &Byte2, 1); // заказываем байт по UART2
//	HAL_TIM_Base_Start_IT(&htim2); // запуск таймера 2
//	HAL_TIM_Base_Start_IT(&htim3); // запуск таймера 3
	LL_SPI_Enable(SPI2);
  LL_SPI_TransmitData16 (SPI2, TX_SPI2[0]);
/* включаем прерывания */
  LL_SPI_EnableIT_RXNE(SPI2);
  LL_SPI_EnableIT_TXE(SPI2);
  LL_SPI_EnableIT_ERR(SPI2);
	HAL_Delay(1);
//	HAL_SPI_TransmitReceive_IT(&hspi2,(uint8_t*)TX_SPI2,(uint8_t*)RX_SPI2,2); // загружаем данные в буфер обмена SPI		
/* иниацилизация SPI в модуляторе */
	for (int i=0; i < 3; i++)
		{
			Reset_EN_TX_MO;	
			HAL_Delay(1);
			Set_EN_TX_MO;
			HAL_Delay(1);
		}
/* иниацилизация SPI в демодуляторе */
	for (int i=0; i < 3; i++)
		{
			Reset_EN_RX_DEM;	
			HAL_Delay(1);
			Set_EN_RX_DEM;
			HAL_Delay(1);
		}
		pinOutput(); // для загрузки аттенюаторов переводим ноги на выход	
/* загружаем аатенюаторы передатчика */
		setAttTX(setings.AttnTX);		
/* загружаем аатенюаторы МШУ */
		setAttLNA(setings.AttnLNA);
/* загружаем потенциометры */
		setPot(0, setings.Pot1Ch1);	
		setPot(1, setings.Pot1Ch2);	
		setPot(2, setings.Pot2Ch1);
		setPot(3, setings.Pot2Ch2);
		setPot(4, setings.Pot3Ch1);
		setPot(5, setings.Pot3Ch2);
		setPot(6, setings.Pot4Ch1);
		setPot(7, setings.Pot4Ch2);		
		
/* загружаем фильтры */	
		setFiltr(1, setings.Filtr1);
		setFiltr(2, setings.Filtr2);
		setFiltr(3, setings.Filtr3);
/* загружаем сдвиговые регистры */
		Reset_EN_BC;
		Reset_EN_TB;
		setShiftReg(0, setings.ChiftReg1);
		setShiftReg(1, setings.ChiftReg2);
/* загружаем модулятор */		
		setModul(30, setings.MCR30);
		setModul(29, 0x80);
		setModul(28, setings.MCR28);
		setModul(27, setings.MCR27);
		setModul(26, 0x00);
		setModul(25, 0x64);
		setModul(24, 0x18);
		setModul(23, 0x70);
		setModul(22, 0x80);
		setModul(21, 0x00);
		setModul(20, 0x00);
		setModul(19, 0x80);
		setModul(18, 0x60);
		setModul(17, 0x00);
		setModul(16, 0x00);
		setModul(15, 0x00);
		setModul(14, 0x80);
		setModul(13, 0xE8);
		setModul(12, 0x18);
		setModul(11, 0x00);		
		setModul(10, setings.MCR10);
		setModul(9, setings.MCR9);
		setModul(8, 0x00);
		setModul(7, setings.MCR7);
		setModul(6, setings.MCR6);
		setModul(5, setings.MCR5);
		setModul(4, 0x01);
		setModul(3, setings.MCR3);
		setModul(2, setings.MCR2);
		setModul(1, setings.MCR1);
		setModul(0, setings.MCR0);
		HAL_Delay(2);
		setModul(29, 0x81);
		setModul(0, setings.MCR0);	
/* загружаем демодулятор */	
		setDemod(30, setings.DCR30);
		setDemod(29, 0x41);
		setDemod(28, setings.DCR28);
		setDemod(27, setings.DCR27);
		setDemod(26, 0x00);
		setDemod(25, 0x70);
		setDemod(24, 0x38);
		setDemod(23, 0x70);
		setDemod(22, 0x00);
		setDemod(21, 0x00);
		setDemod(20, 0x00);
		setDemod(19, 0x00);
		setDemod(18, 0x60);
		setDemod(17, 0x00);
		setDemod(16, 0x00);
		setDemod(15, 0x00);
		setDemod(14, 0x00);
		setDemod(13, 0x08);
		setDemod(12, 0x18);
		setDemod(11, 0x00);
		setDemod(10, setings.DCR10);
		setDemod(9, setings.DCR9);
		setDemod(8, 0x00);
		setDemod(7, setings.DCR7);
		setDemod(6, setings.DCR6);
		setDemod(5, setings.DCR5);
		setDemod(4, 0x01);
		setDemod(3, setings.DCR3);
		setDemod(2, setings.DCR2);
		setDemod(1, setings.DCR1);
		setDemod(0, setings.DCR0);
		HAL_Delay(2);
		setDemod(30, 0x01);
		setDemod(0, setings.DCR0);		
/* проверяем в каком режиме работать */
	if ((GPIOA->IDR & GPIO_IDR_IDR11) && !mode) {mode = 1; flagWork = 1;} // проверяем состояние пина работа/настройка
	if (!(GPIOA->IDR & GPIO_IDR_IDR11) && mode) {mode = 0; flagConfig = 1;}
	if (flagWork) pinInput();					// при переключении в режим "работа" пины на "вход"
	if (flagConfig) pinOutput();			// при переключении в режим "настройка" пины на "выход"
//	MX_IWDG_Init();
//	__enable_irq();										// разрешаем прерывания
	Set_BB_OK; flagBB_OK = 1;  											// выставляем флаг готовности для SPI
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		HAL_IWDG_Refresh(&hiwdg);															// сбрасываем WDT
		if(flagSpi2) ComndProcessing();												// если флаг = 1 то начинаем отработку команд с SPI2
//		if(GPIOC->IDR  &  GPIO_IDR_IDR5)	//ld_tx
//		{
//				Reset_EN_GAIN_RSSI;
//		}
//		else 
//		{			
//				Set_EN_GAIN_RSSI;	
//		}	

//		if(GPIOB->IDR  &  GPIO_IDR_IDR11)	//ld_tx
//		{
//				Reset_EN_GAIN_RX_IQ;
//		}
//		else 
//		{			
//				Set_EN_GAIN_RX_IQ	
//		}			
		if(uart1.rxgap) 
		{
			if(uart1.rxcnt >=2) { Terminal();} 	  // если пришло три байта то переходим к обработке
			uart1.rxgap = 0; uart1.rxcnt = 0; 		// сбрасываем флаг и счетчик символов
		}		
//		if(flagADC) { 
//			rssi = adc[4]; 			// ADC_IN1 
//			power1 = adc[0]; 		// ADC_IN4 выходная мощность на второй антенне
//			power2 = adc[1]; 		// ADC_IN5 выходная мощность на первой антенне
//			ksvn1 = adc[2]; 		// ADC_IN6 КСВН 1-ой антенны
//			ksvn2 = adc[3]; 		// ADC_IN8 КСВН 2-ой антенны	
//			tempr = ((1.43-(adc[5]*0.00080586))/0.0043)+25; 	// вычисление температуры	по формуле из DSh
//			flagADC = 0;				// сбрасываем флаг АЦП
//			flag500Hz_En = 0;		// сбрасываем влаг наличия частоты синхронизации 500 Гц.
//		}		
/* обрабатываем флаг прихода данных от УМ	*/	
		if (flagPwrAmpl)
			{
				if (Byte2&0x10) status |= 0x08; else status &= ~0x08;  // перегрев УМ
				if ((Byte2&0x20)||(Byte2&0x40)) status |= 0x04; else status &= ~0x04;	 // отказ УМ		
				flagPwrAmpl = 0;
			}	
/* проверяем не завис ли сигнал BB_OK	*/	
		if (!flagBB_OK && (T2 > 3)) {Set_BB_OK; flagBB_OK = 1; numrx = 0; } 		
/* сбрасываем счетчик секундного интервала и выставляем флаг для запуска АЦП */
//		if(T1S >= 10000) 
//		{
//			T1S = 0; 
//			if(flag500Hz_En) flag1S = 1; 
//			else startADC();
//		} 
		/* определяем какая антенна подключена */
	AntSw = GPIOD->IDR  &  GPIO_IDR_IDR2;
// если изменилось состояние ANT_LNA	

	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  
	
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_PLL2;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL2_ON;
  RCC_OscInitStruct.PLL2.PLL2MUL = RCC_PLL2_MUL8;
  RCC_OscInitStruct.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV4;
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
  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
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
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**SPI2 GPIO Configuration
  PB12   ------> SPI2_NSS
  PB13   ------> SPI2_SCK
  PB14   ------> SPI2_MISO
  PB15   ------> SPI2_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_12|LL_GPIO_PIN_13|LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_14;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* SPI2 interrupt Init */
  NVIC_SetPriority(SPI2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(SPI2_IRQn);

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_SLAVE;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_HARD_INPUT;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI2, &SPI_InitStruct);
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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 7199;
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
  htim3.Init.Prescaler = 999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 7199;
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
  huart2.Init.BaudRate = 921600;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, EN_TX_IQ_Pin|EN_GAIN_TX_IQ_Pin|EN_GAIN_RX_DEM_Pin|EN_GAIN_RSSI_Pin
                          |EN_RX_IQ_Pin|EN_TB_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EN_BC_Pin|EN_GAIN_RX_IQ_Pin|EN_RX_RSSI_Pin|EN_TX_MOD_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BB_OK_GPIO_Port, BB_OK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EN_RX_DEM_GPIO_Port, EN_RX_DEM_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : EN_TX_IQ_Pin EN_GAIN_TX_IQ_Pin EN_GAIN_RX_DEM_Pin EN_GAIN_RSSI_Pin
                           EN_RX_IQ_Pin EN_TB_Pin */
  GPIO_InitStruct.Pin = EN_TX_IQ_Pin|EN_GAIN_TX_IQ_Pin|EN_GAIN_RX_DEM_Pin|EN_GAIN_RSSI_Pin
                          |EN_RX_IQ_Pin|EN_TB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ANT_FAIL2_Pin ANT_LNA_Pin LD_TX_Pin T010_Pin
                           T020_Pin T040_Pin T080_Pin T160_Pin
                           T320_Pin */
  GPIO_InitStruct.Pin = ANT_FAIL2_Pin|ANT_LNA_Pin|LD_TX_Pin|T010_Pin
                          |T020_Pin|T040_Pin|T080_Pin|T160_Pin
                          |T320_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : TX_ON_Pin */
  GPIO_InitStruct.Pin = TX_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(TX_ON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_BC_Pin EN_GAIN_RX_IQ_Pin EN_RX_RSSI_Pin EN_TX_MOD_Pin */
  GPIO_InitStruct.Pin = EN_BC_Pin|EN_GAIN_RX_IQ_Pin|EN_RX_RSSI_Pin|EN_TX_MOD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GOOD_Pin LD_RX_Pin FAILPA_Pin OVERT_Pin */
  GPIO_InitStruct.Pin = GOOD_Pin|LD_RX_Pin|FAILPA_Pin|OVERT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BB_OK_Pin */
  GPIO_InitStruct.Pin = BB_OK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BB_OK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EN_RX_DEM_Pin */
  GPIO_InitStruct.Pin = EN_RX_DEM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(EN_RX_DEM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FLASH_Pin T_ON_T_Pin */
  GPIO_InitStruct.Pin = FLASH_Pin|T_ON_T_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ANT_SW_Pin */
  GPIO_InitStruct.Pin = ANT_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ANT_SW_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//-----------------------------------------------------------------------
// функции работы с FLASH
//-----------------------------------------------------------------------
void FLASH_Init(void) {
    /* Next commands may be used in SysClock initialization function
       In this case using of FLASH_Init is not obligatorily */
    /* Enable Prefetch Buffer */
    FLASH->ACR |= FLASH_ACR_PRFTBE;
    /* Flash 2 wait state */
}
 
void FLASH_ReadSettings(void) {
    //Read settings
    uint32_t *source_addr = (uint32_t *)MY_FLASH_PAGE_ADDR;
    uint32_t *dest_addr = (void *)&setings;
    for (uint16_t i=0; i<SETTINGS_WORDS; i++) {
        *dest_addr = *(__IO uint32_t*)source_addr;
        source_addr++;
        dest_addr++;
    }
}
 
void FLASH_WriteSettings(void) {
    HAL_FLASH_Unlock();
    FLASH_PageErase(MY_FLASH_PAGE_ADDR);
		CLEAR_BIT(FLASH->CR, FLASH_CR_PER); 
    // Write settings
    uint32_t *source_addr = (void *)&setings;
    uint32_t *dest_addr = (uint32_t *) MY_FLASH_PAGE_ADDR;
    for (uint16_t i=0; i<SETTINGS_WORDS; i++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,(uint32_t)dest_addr, *source_addr);
        source_addr++;
        dest_addr++;
    }
 
    HAL_FLASH_Lock();
}
//----------------------------------------------------------------------
// функция обработки приема байта по UART1 и UART2
//----------------------------------------------------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1)
	{
		uart1.rxtimer=0;																							// сбрасываем таймер
		if(uart1.rxcnt>(6))	uart1.rxcnt=0;														// проверяем не закончился ли буфер		
		RX1[uart1.rxcnt++] = Byte1; 																	// сохраняем принятые данные в буфере	
		HAL_UART_Receive_IT(&huart1, &Byte1, 1);											// заказываем следующий байт		
	}
	if(huart->Instance == USART2)
	{	
		HAL_UART_Receive_IT(&huart2, &Byte2, 1);
		flagPwrAmpl = 1;																							// получен ответ от усилителя мощности
	}
}
//----------------------------------------------------------------------
// функции завершения приема/передачи по SPI2
//----------------------------------------------------------------------
void  SPI2_TX_Callback(void)
{	
//  buf_tx = 0;
//	numtx=numtx+2;
//	if (numtx>2) numtx=0;
	buf_tx = TX_SPI2[2]; buf_tx = (buf_tx << 8) + TX_SPI2[3];
  LL_SPI_TransmitData16(SPI2, buf_tx);
}
void  SPI2_RX_Callback(void)
{ 
//	Set_BB_OK; flagBB_OK = 1;
//	if (T2 > 2) numrx = 0;
  T2 = 0; 											// сбрасываем таймер
	buf_rx = LL_SPI_ReceiveData16(SPI2);
	RX_SPI2[numrx] = (uint8_t)(buf_rx >> 8);
	RX_SPI2[numrx+1] = (uint8_t)buf_rx;	
  numrx=numrx+2;
  if(numrx >2)
  {
    flagSpi2=1;								// выставляем флаг приема посылки из 2-х слов	
//    LL_SPI_DisableIT_TXE(SPI2);    
//    LL_SPI_DisableIT_RXNE(SPI2);
  }
}
//----------------------------------------------------------------------
// функция Terminal (обработка команд по UART1)
//----------------------------------------------------------------------
void Terminal(void)			
{	uart1.rxcnt=0;
	uint16_t chnl = 0;
	switch((uint8_t)RX1[0])
	{	
		case 0x01:
			prntVar();      // печать всех переменных 
			break;
		case 0x02:
			saveVar();			// сохранение всех переменных во FLASH
			HAL_UART_Transmit_DMA(&huart1,(uint8_t*)RX1,3);
			break;
		case 0xC3:			
			chnl = RX1[1] << 8;
			chnl = chnl + RX1[2];
			flagFrqRX = 1; flagFrqTX = 1;
			setFreqChannl( chnl); // установка частотного канала
			HAL_UART_Transmit_DMA(&huart1,(uint8_t*)RX1,3);
			break;	
		case 0x0F:							// установка частоты среза ФНЧ
			if (RX1[1] == RX1[2]) setLPF(RX1[1]);
			HAL_UART_Transmit_DMA(&huart1,(uint8_t*)RX1,3);
			break;
		case 0x78:							// смена режима работы передатчика
			if (RX1[1] == RX1[2]) setOperMode(RX1[1]);
			HAL_UART_Transmit_DMA(&huart1,(uint8_t*)RX1,3);
			break;
		case 0x1E:							// загрузка потенциометров
			setPot(RX1[1], RX1[2]);
			HAL_UART_Transmit_DMA(&huart1,(uint8_t*)RX1,3);
			break;
		case 0x2D:							// загрузка сдвиговых регистров
			setShiftReg(RX1[1], RX1[2]);
			HAL_UART_Transmit_DMA(&huart1,(uint8_t*)RX1,3);
			break;
		case 0x3C:							// загрузка фильтров
			setFiltr(RX1[1], RX1[2]);
			HAL_UART_Transmit_DMA(&huart1,(uint8_t*)RX1,3);
			break;
		case 0x4B:							// загрузка модулятора			
			setModul(RX1[1], RX1[2]);		  
			HAL_UART_Transmit_DMA(&huart1,(uint8_t*)RX1,3);
			break;
		case 0xB4:							// чтение модулятора
			if (RX1[1] == RX1[2]) RX1[2] = readModul(RX1[1]);
			HAL_UART_Transmit_DMA(&huart1,(uint8_t*)RX1,3);
			break;
		case 0x5A:							// загрузка демодулятора
			setDemod(RX1[1], RX1[2]);
			HAL_UART_Transmit_DMA(&huart1,(uint8_t*)RX1,3);
			break;
		case 0xA5:							// чтение демодулятора
			if (RX1[1] == RX1[2])RX1[2] = readDemod(RX1[1]);
			HAL_UART_Transmit_DMA(&huart1,(uint8_t*)RX1,3);
			break;
		case 0x69:							// чтение из EEPROM
			if (RX1[1] == RX1[2])readVar(RX1[1]);
			HAL_UART_Transmit_DMA(&huart1,(uint8_t*)RX1,3);
			break;
		case 0x96:							// запись в EEPROM
			wrtVar(RX1[1], RX1[2]);
			HAL_UART_Transmit_DMA(&huart1,(uint8_t*)RX1,3);
			break;
		case 0xD2:							// загрузка аатенюатора передатчика
			if (RX1[1] == RX1[2])setAttTX(RX1[1]);
			HAL_UART_Transmit_DMA(&huart1,(uint8_t*)RX1,3);
			break;
		case 0xE1:							// загрузка аттенюатора МШУ
			if (RX1[1] == RX1[2])setAttLNA(RX1[1]);
			HAL_UART_Transmit_DMA(&huart1,(uint8_t*)RX1,3);
			break;	
		case 0x87:							// запрос телеметрии
			if (RX1[1] == RX1[2])telemetr(RX1[1]);
			HAL_UART_Transmit_DMA(&huart1,(uint8_t*)RX1,3);
			break;
	}	
}
//----------------------------------------------------------------------
// функция prntVar (вывод всех переменных)
//----------------------------------------------------------------------
void prntVar(void)
{
	uint32_t *source_addr =(void *)&setings;
	uint32_t *dest_addr = (void *)&TX1;
	for (int i=0; i<45; i++) 
	{
      *dest_addr = *source_addr;
      source_addr++;
      dest_addr++;
	}	
	HAL_UART_Transmit_DMA(&huart1,(uint8_t*)TX1,45);
}
//----------------------------------------------------------------------
// функция saveVar (сохранение всех переменных во FLASH)
//----------------------------------------------------------------------
void saveVar(void)
{	
	setings.cntWrFl++; 		// увеличиваем счетчик количества записей
	FLASH_WriteSettings();
	HAL_Delay(100);
}
//----------------------------------------------------------------------
// функция setFreqChannl (установка частотного канала)
//----------------------------------------------------------------------
int setFreqChannl(uint16_t uiNum)
{
//	unsigned int uiN;
//  uint8_t ucVar2_1, ucVar2_2, ucVar2_3, ucDiv;
//  uint16_t usVar3_1, usVar3_2;
//  unsigned long ulFRAC;		
	setShiftReg(0, 0);		// записываем в 0-й сдвиговый регистр число 0x00
/*устанавливаем сдвиговый регистр по частоте 133-636 Мгц	*/		
			if (flagFrqTX) 
			{	
				if (frqTX != uiNum)      // если частота новая то загрузка частоты в передатчик
				{
					setFreqTX(uiNum);
					flagFrqTX	= 0;
				}
				else if(flagSleepPA)
					{
/* загрузка частотной литеры в УМ */
					uint8_t FriqLit = 0;
					if (uiNum <=466) FriqLit = 1;
					else if (uiNum >= 467 && uiNum <=493) FriqLit = 2;
						else if (uiNum >= 494 && uiNum <=520) FriqLit = 3;
							else if (uiNum >= 521 && uiNum <=547) FriqLit = 4;
								else if (uiNum >= 548 && uiNum <=574) FriqLit = 5;
									else if (uiNum >= 575 && uiNum <=601) FriqLit = 6;
										else if (uiNum >= 602 && uiNum <=628) FriqLit = 7;
											else if (uiNum >= 629 && uiNum <=655) FriqLit = 8;
												else if (uiNum >= 656 && uiNum <=682) FriqLit = 9;
													else if (uiNum >= 683) FriqLit = 10;						
						HAL_UART_Transmit_DMA(&huart2,&FriqLit,1);		// отправляем в УМ номер частотной литеры
						flagSleepPA = 0;
					}			
			}
			if (flagFrqRX) {	if (frqRX != uiNum){setFreqRX(uiNum);flagFrqRX = 0;	 // загрузка частоты в приемник
/* вычисляем код номера канала */
				if (uiNum >= 486 && uiNum <=511) usKodNumChanNew = 0;
				else if (uiNum >= 512 && uiNum <=523) usKodNumChanNew = 1;
					else if (uiNum >= 524 && uiNum <=535) usKodNumChanNew = 2;
						else if (uiNum >= 536 && uiNum <=547) usKodNumChanNew = 3;
							else if (uiNum >= 548 && uiNum <=559) usKodNumChanNew = 4;
								else if (uiNum >= 560 && uiNum <=571) usKodNumChanNew = 5;
									else if (uiNum >= 572 && uiNum <=583) usKodNumChanNew = 6;
										else if (uiNum >= 584 && uiNum <=595) usKodNumChanNew = 7;
											else if (uiNum >= 596 && uiNum <=607) usKodNumChanNew = 8;
												else if (uiNum >= 608 && uiNum <=619) usKodNumChanNew = 9;
													else if (uiNum >= 620 && uiNum <=631) usKodNumChanNew = 10;
														else if (uiNum >= 632 && uiNum <=643) usKodNumChanNew = 11;
															else if (uiNum >= 644 && uiNum <=655) usKodNumChanNew = 12;
																else if (uiNum >= 656 && uiNum <=667) usKodNumChanNew = 13;
																	else if (uiNum >= 668 && uiNum <=679) usKodNumChanNew = 14;
																		else if (uiNum >= 680 && uiNum <=706) usKodNumChanNew = 15;
																			else if (uiNum < 486 || uiNum > 706) usKodNumChanNew = 32;
															
	
            if(usKodNumChanOld != usKodNumChanNew)
            {
                usKodNumChanOld = usKodNumChanNew;
								usKodNumChanNew = usKodNumChanNew^0x0F;
							if (!(setings.antLNA & 0x01))
							{
								usKodNumChanNew = usKodNumChanNew + 16;
							}
							else 	usKodNumChanNew = usKodNumChanNew + AntLna;
                setShiftReg(1, usKodNumChanNew);     // загрузка сдвигового регистра №1			           
            }
					}
				}															
    return 0;        	
}
//----------------------------------------------------------------------
// функция setFreqTX (установка частоты передатчика)
//----------------------------------------------------------------------
void setFreqTX(uint16_t uiNum)
{
unsigned int uiN;
uint8_t ucVar2_1, ucVar2_2, ucVar2_3, ucDiv;
uint16_t usVar3_1, usVar3_2;
/* загрузка частотной литеры в УМ */
				uint8_t FriqLit = 0;
				if (uiNum <=466) FriqLit = 1;
				else if (uiNum >= 467 && uiNum <=493) FriqLit = 2;
					else if (uiNum >= 494 && uiNum <=520) FriqLit = 3;
						else if (uiNum >= 521 && uiNum <=547) FriqLit = 4;
							else if (uiNum >= 548 && uiNum <=574) FriqLit = 5;
								else if (uiNum >= 575 && uiNum <=601) FriqLit = 6;
									else if (uiNum >= 602 && uiNum <=628) FriqLit = 7;
										else if (uiNum >= 629 && uiNum <=655) FriqLit = 8;
											else if (uiNum >= 656 && uiNum <=682) FriqLit = 9;
												else if (uiNum >= 683) FriqLit = 10;
HAL_UART_Transmit_DMA(&huart2,&FriqLit,1);		// отправляем в УМ номер частотной литеры	
flagSleepPA = 0;	
unsigned long ulFRAC;
setModul(29, 0x80);		// записываем в 29 регистр число 0x80
/*загрузка по частоте модулятора */
            if(uiNum > 99 && uiNum < 145)
            {
                ucDiv = 4;
                ucVar2_1 = (uiNum-100) / 5;//Целая часть
                ucVar2_2 = uiNum % 5;//дробная часть
                ucVar2_3 = 0;
                switch (ucVar2_2)
                {
                    case 0:
                        ulFRAC = 0;
                    break;
                    case 1:
                        ulFRAC = 26843546;
                    break;
                    case 2:
                        ulFRAC = 20132659;
                        ucVar2_3 = 1;
                    break;
                    case 3:
                        ulFRAC = 13421773;
                        ucVar2_3 = 2;
                    break;
                    case 4:
                        ulFRAC = 6710886;
                        ucVar2_3 = 3;
                    break;
                    default:
                    break;
                }
                uiN = ucVar2_1*4 + 80 + ucVar2_3;
            }
					//загрузка по частоте
            else if(uiNum > 144 && uiNum < 289)
            {
                ucDiv = 3;
                ucVar2_1 = (uiNum-145) / 5;//Целая часть
                ucVar2_2 = uiNum % 5;//дробная часть
                ucVar2_3 = 0;
                switch (ucVar2_2)
                {
                    case 0:
                        ulFRAC = 0;
                    break;
                    case 1:
                        ulFRAC = 13421773;
                    break;
                    case 2:
                        ulFRAC = 26843546;
                    break;
                    case 3:
                        ulFRAC = 6710886;
                        ucVar2_3 = 1;
                    break;
                    case 4:
                        ulFRAC = 20132659;
                        ucVar2_3 = 1;
                    break;
                    default:
                    break;
                }
                uiN = ucVar2_1*2 + 58 + ucVar2_3;
            }
			//загрузка по частоте
            else if(uiNum > 288 && uiNum < 578)
            {
                ucDiv = 2;
                ucVar2_1 = (uiNum-285) / 5;//Целая часть
                ucVar2_2 = uiNum % 5;//дробная часть
                switch (ucVar2_2)
                {
                    case 0:
                        ulFRAC = 0;
                    break;
                    case 1:
                        ulFRAC = 6710886;
                    break;
                    case 2:
                        ulFRAC = 13421773;
                    break;
                    case 3:
                        ulFRAC = 20132659;
                    break;
                    case 4:
                        ulFRAC = 26843546;
                    break;
                    default:
                    break;
                }
                uiN = ucVar2_1 + 57;
            }
			//загрузка по частоте модулятора
            else if(uiNum > 577)
            {
                ucDiv = 1;
                ucVar2_1 = (uiNum-570) / 10;//Целая часть
                ucVar2_2 = uiNum % 10;//дробная часть
                switch (ucVar2_2)
                {
                    case 0:
                        ulFRAC = 0;
                    break;
                    case 1:
                        ulFRAC = 3355443;
                    break;
                    case 2:
                        ulFRAC = 6710886;
                    break;
                    case 3:
                        ulFRAC = 10066330;
                    break;
                    case 4:
                        ulFRAC = 13421773;
                    break;
                    case 5:
                        ulFRAC = 16777216;
                    break;
                    case 6:
                        ulFRAC = 20132659;
                    break;
                    case 7:
                        ulFRAC = 23488102;
                    break;
                    case 8:
                        ulFRAC = 26843546;
                    break;
                    case 9:
                        ulFRAC = 30198988;
                    break;
                    default:
                    break;
                }
                uiN = ucVar2_1 + 57;
            }

            usVar3_1 = 28;//адрес регистра
            usVar3_2 = 0x08 + ucDiv;//данные регистра						
            setModul (usVar3_1,usVar3_2);            
            
            usVar3_1 = 6;//адрес регистра
            usVar3_2 = uiN;//данные регистра
            setModul (usVar3_1,usVar3_2);            

            usVar3_1 = 3;//адрес регистра
            usVar3_2 = (ulFRAC >> 24) & 0xff;//данные регистра
            setModul (usVar3_1,usVar3_2);
            
            usVar3_1 = 2;//адрес регистра
            usVar3_2 = (ulFRAC >> 16) & 0xff;//данные регистра
            setModul (usVar3_1,usVar3_2);
            
            usVar3_1 = 1;//адрес регистра
            usVar3_2 = (ulFRAC >> 8) & 0xff;//данные регистра
						setModul (usVar3_1,usVar3_2);
            
            usVar3_1 = 0;//адрес регистра
            usVar3_2 = ulFRAC & 0xff;//данные регистра
            setModul (usVar3_1,usVar3_2);
  //7- загрузка регистра в модуляторе CR29 = 0x81
            usVar3_1 = 29;//адрес регистра
            usVar3_2 = 0x81;//данные регистра
            setModul (usVar3_1,usVar3_2);						
						frqTX = uiNum;						
}
//----------------------------------------------------------------------
// функция setFreqRX (установка частоты приемника)
//----------------------------------------------------------------------
void setFreqRX(uint16_t uiNum)
{
	unsigned int uiN;
  uint8_t ucVar2_1, ucVar2_2, ucDiv;
  uint16_t usVar3_1, usVar3_2;
  unsigned long ulFRAC;	
//загружаем демодулятор по частоте от 100 до 125, делитель на 16
			if(uiNum > 99 && uiNum < 126)
            {
                ucDiv = 3;
                ucVar2_1 = ((uiNum-100)*4) / 5;//Целая часть
                ucVar2_2 = uiNum % 5;//дробная часть
                switch (ucVar2_2)
                {
                    case 0:
                        ulFRAC = 0;
                    break;
                    case 1:
                        ulFRAC = 26843546;
                    break;
                    case 2:
                        ulFRAC = 20132659;
                    break;
                    case 3:
                        ulFRAC = 13421773;
                    break;
                    case 4:
                        ulFRAC = 6710886;
                    break;
                    default:
                    break;
                }
                uiN = ucVar2_1 + 50;
            }	
//загружаем демодулятор по частоте от 126 до 250, делитель на 8
				else if(uiNum > 125 && uiNum < 251)
            {
                ucDiv = 2;
                ucVar2_1 = ((uiNum-125)*2) / 5;//Целая часть
                ucVar2_2 = uiNum % 5;//дробная часть
                switch (ucVar2_2)
                {
                    case 0:
                        ulFRAC = 0;
                    break;
                    case 1:
                        ulFRAC = 13421773;
                    break;
                    case 2:
                        ulFRAC = 26843546;
                    break;
                    case 3:
                        ulFRAC = 6710886;
                    break;
                    case 4:
                        ulFRAC = 20132659;
                    break;
                    default:
                    break;
                }
                uiN = ucVar2_1 + 50;
            }
						//загружаем демодулятор по частоте от 251 до 500, делитель на 4
            else if(uiNum > 250 && uiNum < 501)
            {
                ucDiv = 1;
                ucVar2_1 = (uiNum-250) / 5;//Целая часть
                ucVar2_2 = uiNum % 5;//дробная часть
                switch (ucVar2_2)
                {
                    case 0:
                        ulFRAC = 0;
                    break;
                    case 1:
                        ulFRAC = 6710886;
                    break;
                    case 2:
                        ulFRAC = 13421773;
                    break;
                    case 3:
                        ulFRAC = 20132659;
                    break;
                    case 4:
                        ulFRAC = 26843546;
                    break;
                    default:
                    break;
                }
                uiN = ucVar2_1 + 50;
            }
			//загружаем демодулятор по частоте от 500, делитель на 2
            else
            {
                ucDiv = 0;
                ucVar2_1 = (uiNum-500) / 10;//Целая часть
                ucVar2_2 = uiNum % 10;//дробная часть
                switch (ucVar2_2)
                {
                    case 0:
                        ulFRAC = 0;
                    break;
                    case 1:
                        ulFRAC = 3355443;
                    break;
                    case 2:
                        ulFRAC = 6710886;
                    break;
                    case 3:
                        ulFRAC = 10066330;
                    break;
                    case 4:
                        ulFRAC = 13421773;
                    break;
                    case 5:
                        ulFRAC = 16777216;
                    break;
                    case 6:
                        ulFRAC = 20132659;
                    break;
                    case 7:
                        ulFRAC = 23488102;
                    break;
                    case 8:
                        ulFRAC = 26843546;
                    break;
                    case 9:
                        ulFRAC = 30198988;
                    break;
                    default:
                    break;
                }
                uiN = ucVar2_1 + 50;
            }
            
            usVar3_1 = 28;//адрес регистра
            usVar3_2 = ucDiv + 0x08;//данные регистра
            setDemod (usVar3_1,usVar3_2);            

            usVar3_1 = 6;//адрес регистра
            usVar3_2 = uiN;//данные регистра
            setDemod (usVar3_1,usVar3_2);
          
            usVar3_1 = 3;//адрес регистра
            usVar3_2 = (ulFRAC >> 24) & 0xff;//данные регистра
            setDemod (usVar3_1,usVar3_2);            
            
            usVar3_1 = 2;//адрес регистра
            usVar3_2 = (ulFRAC >> 16) & 0xff;//данные регистра
            setDemod (usVar3_1,usVar3_2);
            
            usVar3_1 = 1;//адрес регистра
            usVar3_2 = (ulFRAC >> 8) & 0xff;//данные регистра
            setDemod (usVar3_1,usVar3_2);            
            
            usVar3_1 = 0;//адрес регистра
            usVar3_2 = ulFRAC & 0xff;//данные регистра
            setDemod (usVar3_1,usVar3_2);
			frqRX = uiNum;			
}
//----------------------------------------------------------------------
// функция setLPF (установка полосы ФНЧ)
//----------------------------------------------------------------------
void setLPF(char FrLPF)
{	
	setFiltr(1, FrLPF);	
	setFiltr(2, FrLPF);	
	setFiltr(3, FrLPF);	
	frqLpf = FrLPF;
}
//----------------------------------------------------------------------
// функция setOperMode (установка режима работы)
//----------------------------------------------------------------------
void setOperMode(char mode)
{	
	if ((uint8_t) mode == 0xAA)
	{
		setModul(29, 0x80);		
		uint8_t cmd = 0x00;
		HAL_UART_Transmit_DMA(&huart2,(uint8_t*)cmd,1);
		T2 = 0;
		HAL_TIM_Base_Start_IT(&htim2); // запуск таймера 2	
		while (!flagPwrAmpl && T2 < 200){}
		RX1[1] = Byte2;
		RX1[2] = Byte2;	
		HAL_TIM_Base_Stop_IT(&htim2); // останов таймера 2		
	}
}
//----------------------------------------------------------------------
// функция setPot (установка потенциометров)
//----------------------------------------------------------------------
void setPot(char pot, char data)
{	
	switch ((uint8_t)pot)
	{
		case 0x00:
			Reset_EN_GAIN_RX_IQ;
			TX_SPI1[0] = 0;
			setings.Pot1Ch1 = data;
		break;
		case 0x01:
			Reset_EN_GAIN_RX_IQ;
			TX_SPI1[0] = 1;
			setings.Pot1Ch2 = data;
		break;
		case 0x02:
			Reset_EN_GAIN_RX_DEM;
			TX_SPI1[0] = 0;
			setings.Pot2Ch1 = data;
		break;
		case 0x03:
			Reset_EN_GAIN_RX_DEM;
			TX_SPI1[0] = 1;
			setings.Pot2Ch2 = data;
		break;
		case 0x04:
			Reset_EN_GAIN_RSSI;
			TX_SPI1[0] = 0;
			setings.Pot3Ch1 = data;
		break;
		case 0x05:
			Reset_EN_GAIN_RSSI;
			TX_SPI1[0] = 1;
			setings.Pot3Ch2 = data;
		break;
		case 0x06:
			Reset_EN_GAIN_TX_IQ;
			TX_SPI1[0] = 0;
			setings.Pot4Ch1 = data;
		break;
		case 0x07:
			Reset_EN_GAIN_TX_IQ;
			TX_SPI1[0] = 1;
			setings.Pot4Ch2 = data;
	}
	TX_SPI1[1] = data;
	HAL_SPI_Transmit(&hspi1,(uint8_t*)TX_SPI1,2,1000);
	Set_EN_GAIN_RX_IQ;
	Set_EN_GAIN_RX_DEM;
	Set_EN_GAIN_RSSI;
	Set_EN_GAIN_TX_IQ;
}
//----------------------------------------------------------------------
// функция setShiftReg (запись в сдвиговые регистры)
//----------------------------------------------------------------------
void setShiftReg(char reg, char data)
{
	TX_SPI1[0] = data;
	HAL_SPI_Transmit(&hspi1,(uint8_t*)TX_SPI1,1,1000);
	switch ((uint8_t)reg)
	{
		case 0x01:
			Set_EN_BC;
			DelayTic(200);
			Reset_EN_BC;
//			setings.ChiftReg1 = data;
			break;
		case 0x00:
			Set_EN_TB;
		  DelayTic(200);
			Reset_EN_TB;
//			setings.ChiftReg2 = data;
	}	
}
//----------------------------------------------------------------------
// функция setFiltr (загрузка фильтров)
//----------------------------------------------------------------------
void setFiltr(char filtr, uint8_t data)
{
	switch ((uint8_t)filtr)
	{
		case 1:
			Reset_EN_TX_IQ;
			setings.Filtr1 = data;
		break;
		case 2:
			Reset_EN_RX_IQ;
			setings.Filtr2 = data;
		break;
		case 3:
			Reset_EN_RX_RSSI;
			setings.Filtr3 = data;
	}
		data = (data << 3) & 0xF8; // сдвигаем на 3 разряда и младшие заполняем нулями
// перевернём байт	
	 data = (data & 0x55) << 1 | (data & 0xAA) >> 1;
   data = (data & 0x33) << 2 | (data & 0xCC) >> 2;
   data = (data & 0x0F) << 4 | (data & 0xF0) >> 4; 
	
	TX_SPI1[0] = 0xFF; // если первый бит 1, то записываем данные
	TX_SPI1[1] = data;
	HAL_SPI_Transmit(&hspi1,(uint8_t*)TX_SPI1,2,1000);
	Set_EN_TX_IQ;
	Set_EN_RX_IQ;
	Set_EN_RX_RSSI;
}
//----------------------------------------------------------------------
// функция setModul (загрузка регистров модулятора)
//----------------------------------------------------------------------
void setModul(char reg, char data)
{	
	Reset_EN_TX_MO;
	TX_SPI1[0] = 0xD4;
	TX_SPI1[1] = reg;
	TX_SPI1[2] = data;	
	HAL_SPI_Transmit(&hspi1,(uint8_t*)TX_SPI1,3,1000);
	Set_EN_TX_MO;
//	DelayTic(100);
}
//----------------------------------------------------------------------
// функция readModul (чтение регистров модулятора)
//----------------------------------------------------------------------
uint8_t readModul(char reg)	
{	
	Reset_EN_TX_MO;
	TX_SPI1[0] = 0xD4;
	TX_SPI1[1] = reg;	
	HAL_SPI_Transmit(&hspi1,(uint8_t*)TX_SPI1,2,1000);
	Set_EN_TX_MO;
	TX_SPI1[0] = 0xD5;
	Reset_EN_TX_MO;
	HAL_SPI_TransmitReceive(&hspi1,(uint8_t*)TX_SPI1,(uint8_t*)RX_SPI1,2,1000);
	Set_EN_TX_MO;
	return RX_SPI1[1];
}
//----------------------------------------------------------------------
// функция setDemod (загрузка регистров демодулятора)
//----------------------------------------------------------------------
uint8_t setDemod(char reg, char data)
{	
	TX_SPI1[0] = 0xD4;
	TX_SPI1[1] = reg;
	TX_SPI1[2] = data;
	Reset_EN_RX_DEM;
	HAL_SPI_Transmit(&hspi1,(uint8_t*)TX_SPI1,3,1000);
	Set_EN_RX_DEM;
//	DelayTic(5);
	return 0;
}
//----------------------------------------------------------------------
// функция readDemod (чтение регистров демодулятора)
//----------------------------------------------------------------------
uint8_t readDemod(char reg)
{
	TX_SPI1[0] = 0xD4;
	TX_SPI1[1] = reg;	
	Reset_EN_RX_DEM;
	HAL_SPI_Transmit(&hspi1,(uint8_t*)TX_SPI1,2,1000);
	Set_EN_RX_DEM;
	TX_SPI1[0] = 0xD5;
	Reset_EN_RX_DEM;
	HAL_SPI_TransmitReceive(&hspi1,(uint8_t*)TX_SPI1,(uint8_t*)RX_SPI1,2,1000);
	Set_EN_RX_DEM;
	return RX_SPI1[1];
}
//----------------------------------------------------------------------
// функция wrtVar (ввод значений без записи во Flash)
//----------------------------------------------------------------------
void wrtVar(char addr, char data)	
{	
	switch ((uint8_t) addr)
	{
	case 0x00:
	setings.FR_CH1 = data;	
	return;
	case 0x01:
	setings.FR_CH2 = data;	
	return;
	case 0x02:
	setings.ChiftReg1 = data;	setShiftReg(0, setings.ChiftReg1);
	return;
	case 0x03:
	setings.ChiftReg2 = data;	setShiftReg(1, setings.ChiftReg2);	
	return;
	case 0x04:
	setings.AttnTX = data; setAttTX(setings.AttnTX);	
	return;
	case 0x05:
	setings.MCR30 = data;	
	return;
	case 0x06:
	setings.Pot1Ch1 = data;	setPot(0, setings.Pot1Ch1);		
	return;
	case 0x07:
	setings.Pot2Ch1 = data;	setPot(2, setings.Pot2Ch1);
	return;
	case 0x08:
	setings.AttnLNA = data;	setAttLNA(setings.AttnLNA);	
	return;
	case 0x09:
	setings.Pot1Ch2 = data;	setPot(1, setings.Pot1Ch2);	
	return;
	case 0x0A:
	setings.Pot2Ch2 = data;	setPot(3, setings.Pot2Ch2);		
	return;
	case 0x0B:
	setings.Pot3Ch1 = data;	setPot(4, setings.Pot3Ch1);	
	return;
	case 0x0C:
	setings.Pot3Ch2 = data;	setPot(5, setings.Pot3Ch2);	
	return;
	case 0x0D:
	setings.Pot4Ch1 = data;	setPot(6, setings.Pot4Ch1);		
	return;
	case 0x0E:
	setings.Pot4Ch2 = data;	setPot(7, setings.Pot4Ch2);		
	return;
	case 0x0F:
	setings.Filtr1 = data;	setFiltr(1, setings.Filtr1);	
	return;
	case 0x10:
	setings.Filtr2 = data;	setFiltr(2, setings.Filtr2);	
	return;
	case 0x11:
	setings.Filtr3 = data;	setFiltr(3, setings.Filtr3);	
	return;
	case 0x12:
	setings.MCR28 = data;		
	return;
	case 0x13:
	setings.MCR27 = data;		
	return;
	case 0x14:
	setings.MCR10 = data;		
	return;
	case 0x15:
	setings.MCR9 = data;	
	return;
	case 0x16:
	setings.MCR7 = data;		
	return;
	case 0x17:
	setings.MCR6 = data;		
	return;
	case 0x18:
	setings.MCR5 = data;		
	return;
	case 0x19:
	setings.MCR3 = data;		
	return;
	case 0x1A:
	setings.MCR2 = data;		
	return;
	case 0x1B:
	setings.MCR1 = data;		
	return;
	case 0x1C:
	setings.MCR0 = data;	
	return;
	case 0x1D:
	setings.DCR28 = data;	
	return;
	case 0x1E:
	setings.DCR27 = data;		
	return;
	case 0x1F:
	setings.DCR10 = data;	
	return;
	case 0x20:
	setings.DCR9 = data;		
	return;
	case 0x21:
	setings.DCR7 = data;		
	return;
	case 0x22:
	setings.DCR6 = data;	
	return;
	case 0x23:
	setings.DCR5 = data;		
	return;
	case 0x24:
	setings.DCR3 = data;		
	return;
	case 0x25:
	setings.DCR2 = data;		
	return;
	case 0x26:
	setings.DCR1 = data;		
	return;
	case 0x27:
	setings.DCR0 = data;		
	return;
	case 0x28:
	setings.DCR30 = data;		
	return;
	case 0x29:
	setings.porPwr0 = data;
	return;
	case 0x2A:
	setings.porPwr1 = data;	
	return;
	case 0x2B:
	setings.porKsv10 = data;	
	return;
	case 0x2C:
	setings.porKsv11 = data;	
	return;
	case 0x2D:
	setings.porKsv20 = data;	
	return;
	case 0x2E:
	setings.porKsv21 = data;	
	return;
	case 0x2F:
	setings.antLNA = data;	
	return;
 }
}
//----------------------------------------------------------------------
// функция readVar (чтение переменных из ОЗУ)
//----------------------------------------------------------------------
void readVar(char addr)
{	
	switch ((uint8_t) addr)
	{
	case 0x00:
	RX1[2] = setings.FR_CH1;
	TX_SPI2[3] = setings.FR_CH1;
	return;
	case 0x01:
	RX1[2] = 	setings.FR_CH2;	
	TX_SPI2[3] = setings.FR_CH2;	
	return;
	case 0x02:
	RX1[2] = 	setings.ChiftReg1;
	TX_SPI2[3] = setings.ChiftReg1;
	return;
	case 0x03:
	RX1[2] = 	setings.ChiftReg2;
	TX_SPI2[3] = setings.ChiftReg2;
	return;
	case 0x04:
	RX1[2] = 	setings.AttnTX;
	TX_SPI2[3] = setings.AttnTX;
	return;
	case 0x05:
	RX1[2] = 	setings.MCR30;
	TX_SPI2[3] = setings.MCR30;
	return;
	case 0x06:
	RX1[2] = 	setings.Pot1Ch1;
	TX_SPI2[3] = setings.Pot1Ch1;
	return;
	case 0x07:
	RX1[2] = 	setings.Pot2Ch1;
	TX_SPI2[3] = setings.Pot2Ch1;
	return;
	case 0x08:
	RX1[2] = 	setings.AttnLNA;
	TX_SPI2[3] = setings.AttnLNA;
	return;
	case 0x09:
	RX1[2] = 	setings.Pot1Ch2;
	TX_SPI2[3] = setings.Pot1Ch2;
	return;
	case 0x0A:
	RX1[2] = 	setings.Pot2Ch2;
	TX_SPI2[3] =	setings.Pot2Ch2;
	return;
	case 0x0B:
	RX1[2] = 	setings.Pot3Ch1;
	TX_SPI2[3] = setings.Pot3Ch1;
	return;
	case 0x0C:
	RX1[2] = 	setings.Pot3Ch2;
	TX_SPI2[3] =	setings.Pot3Ch2;
	return;
	case 0x0D:
	RX1[2] = 	setings.Pot4Ch1;
	TX_SPI2[3] =	setings.Pot4Ch1;
	return;
	case 0x0E:
	RX1[2] = 	setings.Pot4Ch2;
	TX_SPI2[3] = setings.Pot4Ch2;
	return;
	case 0x0F:
	RX1[2] = 	setings.Filtr1;
	TX_SPI2[3] = setings.Filtr1;
	return;
	case 0x10:
	RX1[2] = 	setings.Filtr2;
	TX_SPI2[3] =	setings.Filtr2;
	return;
	case 0x11:
	RX1[2] = 	setings.Filtr3;
	TX_SPI2[3] =		setings.Filtr3;	
	return;
	case 0x12:
	RX1[2] = 	setings.MCR28;
	TX_SPI2[3] =	setings.MCR28;
	return;
	case 0x13:
	RX1[2] = 	setings.MCR27;
	TX_SPI2[3] = setings.MCR27;
	return;
	case 0x14:
	RX1[2] = 	setings.MCR10;
	TX_SPI2[3] = setings.MCR10;
	return;
	case 0x15:
	RX1[2] = 	setings.MCR9;
	TX_SPI2[3] = setings.MCR9;
	return;
	case 0x16:
	RX1[2] = 	setings.MCR7;
	TX_SPI2[3] =	setings.MCR7;
	return;
	case 0x17:
	RX1[2] = 	setings.MCR6;
	TX_SPI2[3] =	setings.MCR6;
	return;
	case 0x18:
	RX1[2] = 	setings.MCR5;
	TX_SPI2[3] =	setings.MCR5;
	return;
	case 0x19:
	RX1[2] = 	setings.MCR3;
	TX_SPI2[3] =	setings.MCR3;	
	return;
	case 0x1A:
	RX1[2] = 	setings.MCR2;
	TX_SPI2[3] =	setings.MCR2;
	return;
	case 0x1B:
	RX1[2] = 	setings.MCR1;
	TX_SPI2[3] =	setings.MCR1;
	return;
	case 0x1C:
	RX1[2] = 	setings.MCR0;
	TX_SPI2[3] =	setings.MCR0;
	return;
	case 0x1D:
	RX1[2] = 	setings.DCR28;
	TX_SPI2[3] =	setings.DCR28;
	return;
	case 0x1E:
	RX1[2] = 	setings.DCR27;
	TX_SPI2[3] =	setings.DCR27;
	return;
	case 0x1F:
	RX1[2] = 	setings.DCR10;
	TX_SPI2[3] =	setings.DCR10;
	return;
	case 0x20:
	RX1[2] = 	setings.DCR9;
	TX_SPI2[3] =	setings.DCR9;
	return;
	case 0x21:
	RX1[2] = 	setings.DCR7;
	TX_SPI2[3] =	setings.DCR7;
	return;
	case 0x22:
	RX1[2] = 	setings.DCR6;
	TX_SPI2[3] =	setings.DCR6;
	return;
	case 0x23:
	RX1[2] = 	setings.DCR5;
	TX_SPI2[3] =	setings.DCR5;
	return;
	case 0x24:
	RX1[2] = 	setings.DCR3;
	TX_SPI2[3] =	setings.DCR3;	
	return;
	case 0x25:
	RX1[2] = 	setings.DCR2;
	TX_SPI2[3] =	setings.DCR2;
	return;
	case 0x26:
	RX1[2] = 	setings.DCR1;
	TX_SPI2[3] =	setings.DCR1;
	return;
	case 0x27:
	RX1[2] = 
	TX_SPI2[3] =	setings.DCR0;
	return;
	case 0x28:
	RX1[2] = 	setings.DCR30;
	TX_SPI2[3] =	setings.DCR30;
	return;
	case 0x29:
	RX1[2] = 	setings.porPwr0;
	TX_SPI2[3] =	setings.porPwr0;
	return;
	case 0x2A:
	RX1[2] = 	setings.porPwr1;
	TX_SPI2[3] =	setings.porPwr1;
	return;
	case 0x2B:
	RX1[2] = 	setings.porKsv10;
	TX_SPI2[3] =	setings.porKsv10;
	return;
	case 0x2C:
	RX1[2] = 	setings.porKsv11;
	TX_SPI2[3] =	setings.porKsv11;
	return;
	case 0x2D:
	RX1[2] = 	setings.porKsv20;
	TX_SPI2[3] =	setings.porKsv20;
	return;
	case 0x2E:
	RX1[2] = 	setings.porKsv21;
	TX_SPI2[3] =	setings.porKsv21;
	return;	
	case 0x2F:
	RX1[2] = setings.antLNA;
	TX_SPI2[3] =	setings.antLNA;
	return;	
	}		
}
//----------------------------------------------------------------------
// функция setAttTX (установка аттенюаторов передатчика)
//----------------------------------------------------------------------
void setAttTX(char data)
{
	uint32_t mask = ((data<<6) | (~data<<22))&0x0FC00FC0; // вычисляем маску для записи в PC6...PC11
	GPIOC->BSRR = mask;	
}
//----------------------------------------------------------------------
// функция setAttLNA (установка аттенюаторов МШУ)
//----------------------------------------------------------------------
void setAttLNA(char data)
{	
	uint32_t mask = ((data<<8) | (~data<<24))&0x03000300; // вычисляем маску для записи в PB8,PB9
	GPIOB->BSRR = mask;	
}

//----------------------------------------------------------------------
// функция telemetr (вывод телеметрии)
//----------------------------------------------------------------------
void telemetr(char data)
{	
	switch ((uint8_t)data){
		case 0x01: {					// регистр статуса
		break;	
		}
		case 0x02: {					//  потенц. №1 канал 1, канал 2
		RX1[1] = 	setings.Pot1Ch1;
		RX1[2] = 	setings.Pot1Ch2;	
		break;	
		}
		case 0x03: {
		RX1[1] = 	setings.Pot2Ch1;
		RX1[2] = 	setings.Pot2Ch2;	
		break;	
		}
		case 0x04: {
		RX1[1] = 	setings.Pot3Ch1;
		RX1[2] = 	setings.Pot3Ch2;		
		break;	
		}
		case 0x05: {
		RX1[1] = 	setings.Pot4Ch1;
		RX1[2] = 	setings.Pot4Ch2;	
		break;	
		}
		case 0x0B: {					// сдвиговый регистр 1
		RX1[2] = 	setings.ChiftReg1;		
		break;	
		}
		case 0x0C: {					// сдвиговый регистр 2
		RX1[2] = 	setings.ChiftReg2;	
		break;	
		}
		case 0x0D: {									
		RX1[2] = 	setings.Filtr1;	
		break;	
		}
		case 0x0E: {									
		RX1[2] = 	setings.Filtr2;
		break;	
		}
		case 0x0F: {									
		RX1[2] = 	setings.Filtr3;	
		break;	
		}
		case 0x10: {									// выдать RSSI
//		startADC();
//		while (!flagADC) {};
//		flagADC = 0;									// сбрасываем флаг АЦП	
		RX1[1] = (uint8_t) (adc[0] >> 8);
		RX1[2] = (uint8_t) adc[0];
		break;	
		}
		case 0x11: {									// выдать выходную мощность
//		startADC();
//		while (!flagADC) {};			
//		flagADC = 0;				// сбрасываем флаг АЦП	
		RX1[1] = (uint8_t) (adc[2] >> 8);
		RX1[2] = (uint8_t) adc[2];			
		break;	
		}
		case 0x12: {									// выдать КСВН
//		startADC();
//		while (!flagADC) {};		
//		flagADC = 0;				// сбрасываем флаг АЦП	
		RX1[1] = (uint8_t) (adc[3] >> 8);
		RX1[2] = (uint8_t) adc[3];
		break;	
		}
		case 0x13: {									// выдать температуру
//		startADC();
//		while (!flagADC) {};
//		flagADC = 0;									// сбрасываем флаг АЦП	
		tempr = ((1.43-(adc[5]*0.00080586))/0.0043)+25; 	// формула вычисления температуры	
		RX1[2] = (uint8_t) tempr;
		break;	
		}				
	}
	HAL_UART_Transmit_DMA(&huart1,(uint8_t*)RX1,3);
}
//----------------------------------------------------------------------
// функция telemetrSPI (вывод телеметрии через SPI)
//----------------------------------------------------------------------
void telemetrSPI(char data)
{
	int i = 0;
	switch ((uint8_t)data){
		case 0x00:		// статус
		TX_SPI2[1] = 0x00;
		TX_SPI2[2] = status >> 8;
		TX_SPI2[3] = (uint8_t) status;
		break;
		
		case 0x01:		//температура платы УВЧ
		startADC();	
		TX_SPI2[1] = 0x01;
		while (!flagADC);
		flagADC = 0;				// сбрасываем флаг АЦП
//		rssi = adc[4]; 			// ADC_IN1 
//		power1 = adc[0]; 		// ADC_IN4 выходная мощность на второй антенне
//		power2 = adc[1]; 		// ADC_IN5 выходная мощность на первой антенне
//		ksvn1 = adc[2]; 		// ADC_IN6 КСВН 1-ой антенны
//		ksvn2 = adc[3]; 		// ADC_IN8 КСВН 2-ой антенны	
		TX_SPI2[2] = 0;
		tempr = ((1.43-(adc[5]*0.00080586))/0.0043)+25;
		TX_SPI2[3] = (int8_t)tempr;
		break;
		
		case 0x02: 		// температура УМ
//		startADC();	
		TX_SPI2[1] = 0x02;
//		while (!flagADC) DelayTic(10);
//		flagADC = 0;				// сбрасываем флаг АЦП		
		TX_SPI2[2] = 0;
		tempr = ((1.43-(adc[5]*0.00080586))/0.0043)+25;
		TX_SPI2[3] = (int8_t)tempr;	
		break;
		
		case 0x03:		// КСВН антенны	
		startADC();	
		TX_SPI2[1] = 0x03;		
		while (!flagADC);		
		flagADC = 0;									// сбрасываем флаг АЦП
		if(AntSw)  i = 4; else i = 3;
		TX_SPI2[2] = adc[i] >> 8;
		TX_SPI2[3] = (uint8_t)adc[i];
		break;
		
		case 0x04:		// мощность на входе антенны
//		startADC();	
		TX_SPI2[1] = 0x04;		
//		while (!flagADC) DelayTic(10);		
//		flagADC = 0;				// сбрасываем флаг АЦП
		if(AntSw)  i = 2; else i = 1;
		TX_SPI2[2] = adc[i] >> 8;
		TX_SPI2[3] = (uint8_t)adc[i];
		break;
		
		case 0x05:		// температура измерителя КСВН
//		startADC();	
		TX_SPI2[1] = 0x05;
//		while (!flagADC) DelayTic(10);	
		TX_SPI2[2] = 0;
//		tempr = ((1.43-(adc[5]*0.00080586))/0.0043)+25;
		TX_SPI2[3] = (int8_t)tempr;	
		break;
		
		case 0x10:		// установленная частота
		TX_SPI2[1] = 0x10;
		if(frqTX == frqRX)
			{				
			TX_SPI2[2] = frqTX >> 8;
			TX_SPI2[3] = (uint8_t) frqTX;
			}
			else {TX_SPI2[2] = 0; TX_SPI2[3] = 0;}
		break;
			
		case 0x11:		// установленная частота среза ФНЧ
		TX_SPI2[1] = 0x11;	
		TX_SPI2[2] = 0;
		TX_SPI2[3] = frqLpf;
		break;
		
		case 0x12:		// установленная частота передатчика в МГц
		TX_SPI2[1] = 0x12;	
		TX_SPI2[2] = frqTX >> 8;
		TX_SPI2[3] = (uint8_t) frqTX;
		break;
		
		case 0x13:		// установленная частота приемника в МГц
		TX_SPI2[1] = 0x13;	
		TX_SPI2[2] = frqRX >> 8;
		TX_SPI2[3] = (uint8_t) frqRX;		
		break;
		
		case 0x76:		// версия ПО программы УМ
		TX_SPI2[1] = RX_SPI2[2];
		TX_SPI2[2] = 0xFF;
		TX_SPI2[3] = 0xFF;		
		break;
		 
		case 0x77:		// версия ПО программы УВЧ
		TX_SPI2[1] = 0x77;
		TX_SPI2[2] = versionPO_1;
		TX_SPI2[3] = versionPO_2;
		break;
		default:
		TX_SPI2[1] = RX_SPI2[2];
		TX_SPI2[2] = 0xFF;
		TX_SPI2[3] = 0xFF;	
	}
}

//----------------------------------------------------------------------
// функция pinOutput (переключение пинов на выход)
//----------------------------------------------------------------------
void pinOutput(void)
	{ 
//для начала все сбрасываем в ноль
		GPIOC->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6); 
		GPIOC->CRL &= ~(GPIO_CRL_MODE7 | GPIO_CRL_CNF7); 
		GPIOC->CRH &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8); 	
		GPIOC->CRH &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9); 
		GPIOC->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10);
		GPIOC->CRH &= ~(GPIO_CRH_MODE11 | GPIO_CRH_CNF11);
		GPIOB->CRH &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8); 	
		GPIOB->CRH &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9); 
		
// устанавливаем mode - выход 
		GPIOC->CRL |= (0x02 << GPIO_CRL_MODE6_Pos) | (0x00 << GPIO_CRL_CNF6_Pos);		
		GPIOC->CRL |= (0x02 << GPIO_CRL_MODE7_Pos) | (0x00 << GPIO_CRL_CNF7_Pos);	
		GPIOC->CRH |= (0x02 << GPIO_CRH_MODE8_Pos) | (0x00 << GPIO_CRH_CNF8_Pos);	
		GPIOC->CRH |= (0x02 << GPIO_CRH_MODE9_Pos) | (0x00 << GPIO_CRH_CNF9_Pos);
		GPIOC->CRH |= (0x02 << GPIO_CRH_MODE10_Pos) | (0x00 << GPIO_CRH_CNF10_Pos);
		GPIOC->CRH |= (0x02 << GPIO_CRH_MODE11_Pos) | (0x00 << GPIO_CRH_CNF11_Pos);
		GPIOB->CRH |= (0x02 << GPIO_CRH_MODE8_Pos) | (0x00 << GPIO_CRH_CNF8_Pos);	
		GPIOB->CRH |= (0x02 << GPIO_CRH_MODE9_Pos) | (0x00 << GPIO_CRH_CNF9_Pos);		
		flagConfig = 0;	    // обнуляем флаг чтобы не повторять конфигурирование пинов
	}
//----------------------------------------------------------------------
// функция pinInput (переключение пинов на вход)
//----------------------------------------------------------------------
void pinInput(void)
{
//для начала все сбрасываем в ноль
		GPIOC->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6); 
		GPIOC->CRL &= ~(GPIO_CRL_MODE7 | GPIO_CRL_CNF7); 
		GPIOC->CRH &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8); 	
		GPIOC->CRH &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9); 
		GPIOC->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10);
		GPIOC->CRH &= ~(GPIO_CRH_MODE11 | GPIO_CRH_CNF11);
		GPIOB->CRH &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8); 	
		GPIOB->CRH &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9); 
// устанавливаем mode - вход 
		GPIOC->CRL |= (0x00 << GPIO_CRL_MODE6_Pos) | (0x01 << GPIO_CRL_CNF6_Pos);		
		GPIOC->CRL |= (0x00 << GPIO_CRL_MODE7_Pos) | (0x01 << GPIO_CRL_CNF7_Pos);	
		GPIOC->CRH |= (0x00 << GPIO_CRH_MODE8_Pos) | (0x01 << GPIO_CRH_CNF8_Pos);	
		GPIOC->CRH |= (0x00 << GPIO_CRH_MODE9_Pos) | (0x01 << GPIO_CRH_CNF9_Pos);
		GPIOC->CRH |= (0x00 << GPIO_CRH_MODE10_Pos) | (0x01 << GPIO_CRH_CNF10_Pos);
		GPIOC->CRH |= (0x00 << GPIO_CRH_MODE11_Pos) | (0x01 << GPIO_CRH_CNF11_Pos);
		GPIOB->CRH |= (0x00 << GPIO_CRH_MODE8_Pos) | (0x01 << GPIO_CRH_CNF8_Pos);	
		GPIOB->CRH |= (0x00 << GPIO_CRH_MODE9_Pos) | (0x01 << GPIO_CRH_CNF9_Pos);		
		flagWork = 0;	    // обнуляем флаг чтобы не повторять конфигурирование пинов		
}
//----------------------------------------------------------------------
// функция обратного вызова для АЦП 
//----------------------------------------------------------------------
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        flagADC = 1;
    }
}
//----------------------------------------------------------------------
// функция startADC (запуск АЦП)
//----------------------------------------------------------------------
void startADC(void)
{

HAL_ADCEx_Calibration_Start(&hadc1);
HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc, 6); // стартуем АЦП
	
}
//----------------------------------------------------------------------
// функция ComndProcessing (обработка команд от SPI2)
//----------------------------------------------------------------------
void ComndProcessing(void)
{
			switch(RX_SPI2[0])
			{
				case 0x00:
					if (RX_SPI2[1] == 0x00){			// установка частоты приемника
					TX_SPI2[0] = RX_SPI2[0];
					TX_SPI2[1] = RX_SPI2[1];	
					TX_SPI2[2] = RX_SPI2[2];
					TX_SPI2[3] = RX_SPI2[3];					
					}
				break;
				case 0xC3:					
					if (RX_SPI2[1] == 0x3C){			// установка частоты
					for (int i=0; i < 4; i++){TX_SPI2[i] = RX_SPI2[i];}	
					flagFrqRX = 1; flagFrqTX = 1;	
					uint16_t freq = RX_SPI2[2] << 8;
					freq = freq + RX_SPI2[3];
					setFreqChannl(freq); 
					frqTX = freq;
					}
				break;
				case 0xB4:
					if (RX_SPI2[1] == 0x4B){			// установка частоты среза ФНЧ
					setLPF(RX_SPI2[3]);
					for (int i=0; i < 4; i++){TX_SPI2[i] = RX_SPI2[i];}	
					}
				break;
				case 0xA5:
					if (RX_SPI2[1] == 0x5A){			// установка постоянной составляющей на выходе приемника
					setPot(1,RX_SPI2[3]);
					for (int i=0; i < 4; i++){TX_SPI2[i] = RX_SPI2[i];}		
					}
				break;
				case 0xF0:
					if (RX_SPI2[1] == 0x0F){			// передатчик в sleep режим
					uint8_t Sleep = 0;
					HAL_TIM_Base_Start_IT(&htim2); // запуск таймера 2	
					HAL_UART_Transmit_DMA(&huart2,&Sleep,1); // отправляем в УМ значение 0х00 (режим ожидания)
					T2 = 0;	
					while (!flagPwrAmpl && (T2 < 2)) {};
					HAL_TIM_Base_Stop_IT(&htim2); // остановка таймера 2	
					if (Byte2 == 0x80) TX_SPI2[3] = 0;
					else 	TX_SPI2[3] = 0xFF;
					for (int i=0; i < 3; i++){TX_SPI2[i] = RX_SPI2[i];}	
					flagSleepPA = 1;
					}
				break;
				case 0xE0:
					if (RX_SPI2[1] == 0x0E){				// изменение переменных в ОЗУ
					wrtVar(RX_SPI2[2],RX_SPI2[3]);						
					TX_SPI2[0] = 0xE0;
					TX_SPI2[1] = 0x0E;
					TX_SPI2[2] = RX_SPI2[2];
					TX_SPI2[3] = 0x00;				
					}								
				break;
				case 0xAF:
					if (RX_SPI2[1] == 0xFA){				// сохранение все переменных во FLASH
					saveVar();
					for (int i=0; i < 4; i++){TX_SPI2[i] = RX_SPI2[i];}			
					}
				break;
					case 0xE5:
					if (RX_SPI2[1] == 0x5E){			 // чтениее переменных из FLASH
					for (int i=0; i < 3; i++){TX_SPI2[i] = RX_SPI2[i];}		
					readVar(RX_SPI2[2]);
	//				TX_SPI2[3] = RX1[2];
					}
				break;
				case 0x87:
					if (RX_SPI2[1] == 0x78){			// запрос телеметрии
					telemetrSPI(RX_SPI2[2]);
					TX_SPI2[0] = RX_SPI2[0];				
					}
				break;
				case 0x2D:
					if (RX_SPI2[1] == 0xD2){			// установка частоты передатчика
					for (int i=0; i < 4; i++){TX_SPI2[i] = RX_SPI2[i];}		
					uint16_t frqMHz = RX_SPI2[2];
					frqMHz = (frqMHz	<< 8) + RX_SPI2[3];
					if(frqMHz != frqTX) {flagFrqTX = 1; setFreqTX(frqMHz); frqTX = frqMHz;}					
					}
				break;
				case 0x1E:
					if (RX_SPI2[1] == 0xE1){			// установка частоты приемника
					for (int i=0; i < 4; i++){TX_SPI2[i] = RX_SPI2[i];}		
					uint16_t frqMHz = RX_SPI2[2];
					frqMHz = (frqMHz	<< 8) + RX_SPI2[3];
					if (frqMHz != frqRX) {flagFrqRX = 1; setFreqRX(frqMHz); frqRX = frqMHz;}					
					}
				break;
				case 0xAA:
					if (RX_SPI2[1] == 0xAA){			// включение выключения LNA
					if (RX_SPI2[3] & 0x80) 	AntLna = 16;						
					else AntLna = 0;
					usKodNumChanNew = usKodNumChanOld;
					usKodNumChanNew = usKodNumChanNew^0x0F;				
					if (!(setings.antLNA & 0x01))				
							{							
								usKodNumChanNew = usKodNumChanNew + 16;
							}
							else 	
							{								
								usKodNumChanNew = usKodNumChanNew + AntLna;								
							}
                setShiftReg(1, usKodNumChanNew);     // загрузка сдвигового регистра №1		
					TX_SPI2[0] = RX_SPI2[0];
					TX_SPI2[1] = RX_SPI2[1];
					TX_SPI2[2] = RX_SPI2[2];
					TX_SPI2[3] = RX_SPI2[3];							
					}
				break;				
				default:
				TX_SPI2[0] = 0xFF;
				TX_SPI2[1] = 0xFF;
				TX_SPI2[2] = 0xFF;
				TX_SPI2[3] = 0xFF;
			}		
		flagSpi2 = 0; numrx=0;
		buf_tx = TX_SPI2[0];
		buf_tx = (buf_tx << 8) + TX_SPI2[1];
//    LL_SPI_EnableIT_RXNE(SPI2);
//    LL_SPI_EnableIT_TXE(SPI2);	
		LL_SPI_TransmitData16(SPI2, buf_tx);
		Set_BB_OK; flagBB_OK = 1; 											// выставляем флаг готовности			
}
//----------------------------------------------------------------------
// функции DelayTic (задержка)
//----------------------------------------------------------------------
void DelayTic(uint32_t Tic)
{
	for (uint32_t i = 0; i < Tic; i++){}	
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
/*
29.03.22 - введена команда AA (включение/выключение МШУ), отключены все таймеры


*/
