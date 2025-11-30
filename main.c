/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Final Project Code v2 (Optimized & Bug Fixed)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "lcd_i2c.h" 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DHT11_PIN GPIO_PIN_4
#define DHT11_PORT GPIOA
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart3; 

/* USER CODE BEGIN PV */
uint8_t Temp = 0, Rh = 0; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
void DHT11_Start(void);
uint8_t DHT11_Read(void);
void microDelay(uint32_t us);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// --- MIKRO SANIYE GECIKMESI ---
void DWT_Init(void) {
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}

void microDelay(uint32_t us) {
    uint32_t startTick = DWT->CYCCNT;
    // 168 MHz sistem saati icin (1 us = 168 cycle)
    uint32_t delayTicks = us * (SystemCoreClock / 1000000);
    while (DWT->CYCCNT - startTick < delayTicks);
}

// --- DHT11 FONKSIYONLARI ---
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL; // Pull-up genelde modülde vardir
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DHT11_Start(void) {
    Set_Pin_Output(DHT11_PORT, DHT11_PIN);
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 0);
    microDelay(18000); // 18ms bekle
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 1);
    microDelay(20);
    Set_Pin_Input(DHT11_PORT, DHT11_PIN);
}

uint8_t DHT11_Read(void) {
    uint8_t i = 0, j; // DUZELTME 2.1: i degiskeni 0 ile baslatildi!
    for (j = 0; j < 8; j++) {
        while (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))); 
        microDelay(40);
        if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) { 
            i &= ~(1 << (7 - j));
        } else { 
            i |= (1 << (7 - j));
        }
        while ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))); 
    }
    return i;
}

uint32_t ADC_Oku(uint32_t Channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = Channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    uint32_t deger = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return deger;
}

// Ham veriyi 0-500 IAQ skalasina cevirir
int Calculate_IAQ(uint16_t raw_gas) {
    return (raw_gas * 500) / 4095;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  
  DWT_Init(); 

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  
  // LCD ACILIS
  lcd_init();
  lcd_clear();
  lcd_put_cur(0, 0);
  lcd_send_string("Sistem V2.0");
  lcd_put_cur(1, 0);
  lcd_send_string("Baslatiliyor...");
  HAL_Delay(2000);
  lcd_clear();
  /* USER CODE END 2 */

  int sayac = 0; 

  while (1)
  {
    MX_USB_HOST_Process();

    // 1. DHT11 OKUMA 
    DHT11_Start();
    uint8_t Check = 0;
    microDelay(40);
    if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) {
        microDelay(80);
        if ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) Check = 1;
    }
    while ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))); 

    if (Check) {
        Rh = DHT11_Read(); 
        uint8_t Rh_dec = DHT11_Read(); // Okuduk ama kullanmiyoruz
        Temp = DHT11_Read(); 
        uint8_t Temp_dec = DHT11_Read(); // Okuduk ama kullanmiyoruz
        uint8_t Sum = DHT11_Read();      // Checksum
        (void)Rh_dec; (void)Temp_dec; (void)Sum; // Warning engellemek icin
    }

    // 2. ANALOG SENSORLER
    uint16_t isik_raw = ADC_Oku(ADC_CHANNEL_1);  
    uint16_t gaz_raw = ADC_Oku(ADC_CHANNEL_2);   
    uint16_t ses_raw = ADC_Oku(ADC_CHANNEL_3);   

    // 3. DIJITAL SENSORLER (DUZELTME 2.3: Temiz Maskeleme)
    uint8_t hareket_var = (GPIOE->IDR & (1 << 4)) ? 1 : 0; // PE4 -> 0 veya 1
    uint8_t pencere_acik = (GPIOE->IDR & (1 << 5)) ? 1 : 0; // PE5 -> 0 veya 1

    // 4. HESAPLAMALAR
    int iaq_score = Calculate_IAQ(gaz_raw);
    uint8_t silent_mode = (ses_raw > 2500) ? 1 : 0; // Esik degeri

    // 5. FAN VE LED KONTROL MANTIGI
    if (pencere_acik) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0); // Pencere aciksa STOP
        GPIOD->ODR |= (1 << 15); // Mavi LED ON
        GPIOD->ODR &= ~((1 << 12)|(1 << 13)|(1 << 14));
    }
    else {
        GPIOD->ODR &= ~(1 << 15);
        
        int fan_hizi = 0;
        
        if (iaq_score > 300) { // Kirli
            fan_hizi = 1000; // PWM Periodunu 1000 yaptigimiz icin bu %100 Hizdir!
            GPIOD->ODR |= (1 << 14); // Kirmizi
            GPIOD->ODR &= ~((1 << 12)|(1 << 13));
        } else if (iaq_score > 100) { // Orta
            fan_hizi = 600; // %60 Hiz
            GPIOD->ODR |= (1 << 13); // Turuncu
            GPIOD->ODR &= ~((1 << 12)|(1 << 14));
        } else { // Temiz
            if (hareket_var) {
                fan_hizi = 350; // %35 Hiz
                GPIOD->ODR |= (1 << 12); // Yesil
                GPIOD->ODR &= ~((1 << 13)|(1 << 14));
            } else {
                fan_hizi = 0; // STOP
                GPIOD->ODR &= ~((1 << 12)|(1 << 13)|(1 << 14));
            }
        }

        // Silent Mode: Gurultu varsa hizi limitler
        if (silent_mode && fan_hizi > 500) {
            fan_hizi = 500; 
        }

        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, fan_hizi);
    }

    // 6. LCD EKRAN (DUZELTME 2.2: Buffer boyutu artirildi ve bosluklar eklendi)
    char lcd_buf[32]; // Boyut artirildi
    
    if ((sayac++ % 2) == 0) {
        // SAYFA 1
        lcd_put_cur(0, 0);
        // %-16s formati ile satir temizligi yapiliyor
        sprintf(lcd_buf, "T:%dC H:%d%%      ", Temp, Rh); 
        lcd_send_string(lcd_buf);
        
        lcd_put_cur(1, 0);
        if(silent_mode) lcd_send_string("Mode: SILENT    ");
        else if(pencere_acik) lcd_send_string("Win: OPEN       ");
        else lcd_send_string("System: ACTIVE  ");
    } else {
        // SAYFA 2
        lcd_put_cur(0, 0);
        sprintf(lcd_buf, "IAQ:%3d Gas:%4d ", iaq_score, gaz_raw);
        lcd_send_string(lcd_buf);
        
        lcd_put_cur(1, 0);
        sprintf(lcd_buf, "L:%3d S:%3d     ", isik_raw/10, ses_raw/10);
        lcd_send_string(lcd_buf);
    }

    // 7. BLUETOOTH
    char bt_msg[150];
    sprintf(bt_msg, "{\"T\":%d,\"H\":%d,\"IAQ\":%d,\"L\":%d,\"Mic\":%d,\"Fan\":%lu}\n", 
            Temp, Rh, iaq_score, isik_raw, ses_raw, TIM3->CCR1);
    HAL_UART_Transmit(&huart3, (uint8_t*)bt_msg, strlen(bt_msg), 100);

    // DUZELTME 3.1: DHT11 icin bekleme suresi 1 sn yapildi
    HAL_Delay(1000); 
  }
}

// --- AYAR FONKSIYONLARI ---

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);
}

static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi1);
}

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
  HAL_UART_Init(&huart3);
}

static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0; // Islemci hizi yuksekse burayi 83 yapabilirsin (1MHz sayac)
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  // DUZELTME 3.3: Period 1000 yapildi. PWM artik %0-%100 arasi duzgun calisacak.
  htim3.Init.Period = 1000; 
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim3);
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);
  HAL_TIM_PWM_Init(&htim3);
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_MspPostInit(&htim3);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin|Audio_RST_Pin, GPIO_PIN_RESET);
  
  // DHT11 ICIN PA4 BASLANGICTA OUTPUT
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = GPIO_PIN_4; // DHT11
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // DUZELTME 2.3: PIR (PE4) ve Reed (PE5) manuel olarak Input tanimlandi
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL; // Harici direnciniz varsa NOPULL, yoksa PULLUP/DOWN deneyin
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin|Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void) {
  __disable_irq();
  while (1){}
}