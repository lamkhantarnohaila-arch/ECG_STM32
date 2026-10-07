/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   ECG (AD8232 -> ADC1/DMA/TIM2) affiche sur OLED 128x64 en I2C
  *          Base : v13 des camarades, SD / GPS / ILI9341 retires.
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* ---- Acquisition ---- */
#define BUFFER_SIZE   256          // mots 32 bits, DMA circulaire -> demi-buffer = 128 ech = 128 ms
#define HALF_SIZE     (BUFFER_SIZE / 2)

uint32_t adc_buffer[BUFFER_SIZE];
volatile uint8_t dma_half_flag     = 0;
volatile uint8_t dma_complete_flag = 0;

/* ---- Affichage ---- */
#define OLED_W        128
#define OLED_H        64
#define OLED_PAGES    8
#define DECIM         8            // ech. par colonne : 128 col x DECIM = duree affichee (8 -> 1,0 s ; 12 -> 1,5 s ; 16 -> 2,0 s)
#define ECG_GAIN_DIV  12           // counts ADC par pixel (diminuer = signal plus grand)
#define MA_LEN        20           // moyenne glissante : 20 ech = 20 ms = 1 periode du 50 Hz secteur (0 effet si 1)
#define CAP_LEN       (OLED_W * DECIM)   // echantillons par capture (1024 -> 1,02 s)

static uint8_t oled_addr = 0x3C << 1;      // mis a jour par le scan
static uint8_t fb[OLED_W * OLED_PAGES];    // framebuffer 1024 octets

static int16_t  cap[CAP_LEN];              // echantillons apres passe-haut (ligne de base retiree)
static int16_t  fil[CAP_LEN];              // apres moyenne glissante
static uint16_t raw_min = 4095, raw_max = 0;   // pour detecter une saturation ADC
#define HP_A          0.997f       // passe-haut ~0,5 Hz a 1 kHz (retire la derive de ligne de base)
static float    hp_y = 0.0f, hp_x_prev = 0.0f;
static uint8_t  hp_init = 0;
static uint16_t cap_n     = 0;
static uint8_t  capturing = 0;
static uint8_t  leads_seen = 0;            // electrode decrochee pendant la capture
/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN 0 */

/* ======================= Bas niveau OLED (I2C) ======================= */
// ctrl : 0x00 = commandes, 0x40 = donnees
static void OLED_WriteBuf(uint8_t ctrl, const uint8_t *data, uint16_t len)
{
    uint8_t buf[129];
    if (len > 128) len = 128;
    buf[0] = ctrl;
    for (uint16_t i = 0; i < len; i++) buf[i + 1] = data[i];
    HAL_I2C_Master_Transmit(&hi2c1, oled_addr, buf, len + 1, 100);
}

static void OLED_WriteCmd(uint8_t cmd)
{
    OLED_WriteBuf(0x00, &cmd, 1);
}

// Meme init que le test valide (SH1106 ; la commande SSD1306 est ignoree par le SH1106)
static void OLED_Init(void)
{
    HAL_Delay(100);
    OLED_WriteCmd(0xAE);
    OLED_WriteCmd(0xD5); OLED_WriteCmd(0x80);
    OLED_WriteCmd(0xA8); OLED_WriteCmd(0x3F);
    OLED_WriteCmd(0xD3); OLED_WriteCmd(0x00);
    OLED_WriteCmd(0x40);
    OLED_WriteCmd(0xAD); OLED_WriteCmd(0x8B);  // DC-DC (SH1106)
    OLED_WriteCmd(0x8D); OLED_WriteCmd(0x14);  // charge pump (SSD1306)
    OLED_WriteCmd(0xA1);
    OLED_WriteCmd(0xC8);
    OLED_WriteCmd(0xDA); OLED_WriteCmd(0x12);
    OLED_WriteCmd(0x81); OLED_WriteCmd(0xCF);
    OLED_WriteCmd(0xD9); OLED_WriteCmd(0xF1);
    OLED_WriteCmd(0xDB); OLED_WriteCmd(0x40);
    OLED_WriteCmd(0xA4);
    OLED_WriteCmd(0xA6);
    OLED_WriteCmd(0xAF);
}

/* ======================= Framebuffer + rafraichissement ======================= */
// Envoie tout le framebuffer, page par page (decalage de 2 colonnes du SH1106)
static void OLED_Refresh(void)
{
    for (uint8_t p = 0; p < OLED_PAGES; p++)
    {
        uint8_t cmd[4] = { 0x00, (uint8_t)(0xB0 + p), 0x02, 0x10 };  // page, colonne 2
        HAL_I2C_Master_Transmit(&hi2c1, oled_addr, cmd, 4, 100);
        OLED_WriteBuf(0x40, &fb[p * OLED_W], OLED_W);
    }
}

static void fb_clear_column(uint8_t x)
{
    for (uint8_t p = 0; p < OLED_PAGES; p++) fb[p * OLED_W + x] = 0;
}

static void fb_vline(uint8_t x, int16_t y0, int16_t y1)
{
    for (int16_t y = y0; y <= y1; y++)
        fb[(y >> 3) * OLED_W + x] |= (uint8_t)(1u << (y & 7));
}

/* ======================= Capture puis affichage ======================= */
// Filtre passe-haut ~0,5 Hz (standard ECG) : supprime la derive lente
// (contact d'electrode, respiration, mouvements). Il tourne en permanence.
static inline int16_t hp_step(uint16_t x)
{
    if (!hp_init) { hp_x_prev = (float)x; hp_y = 0.0f; hp_init = 1; }
    hp_y = HP_A * (hp_y + (float)x - hp_x_prev);
    hp_x_prev = (float)x;
    return (int16_t)hp_y;
}

// Pendant la capture on ne fait QUE copier les echantillons (aucun acces I2C) :
// pas de bloc perdu, pas de perturbation de l'ADC. L'affichage est fait ensuite.
static void render_capture(void)
{
    uint32_t t0 = HAL_GetTick();

    /* 1) Moyenne glissante + statistiques de diagnostic */
    int32_t  sum = 0, total = 0;
    int32_t  jump_max = 0;
    uint16_t jump_idx = 0;
    int16_t  vmin = 32767, vmax = -32768;

    for (int i = 0; i < CAP_LEN; i++)
    {
        sum += cap[i];
        if (i >= MA_LEN) sum -= cap[i - MA_LEN];
        int32_t n = (i + 1 < MA_LEN) ? (i + 1) : MA_LEN;
        fil[i] = (int16_t)(sum / n);
        total += fil[i];

        if (cap[i] < vmin) vmin = cap[i];
        if (cap[i] > vmax) vmax = cap[i];
        if (i > 0) {
            int32_t d = (int32_t)cap[i] - (int32_t)cap[i - 1];
            if (d < 0) d = -d;
            if (d > jump_max) { jump_max = d; jump_idx = (uint16_t)i; }
        }
    }
    int32_t mean = total / CAP_LEN;

    /* 2) Trace dans le framebuffer */
    memset(fb, 0, sizeof(fb));
    int16_t prev_y = -1;

    for (int x = 0; x < OLED_W; x++)
    {
        int16_t ymin = OLED_H - 1, ymax = 0, y_last = OLED_H / 2;

        for (int k = 0; k < DECIM; k++)
        {
            int32_t y = (OLED_H / 2) - (((int32_t)fil[x * DECIM + k] - mean) / ECG_GAIN_DIV);
            if (leads_seen) y = OLED_H / 2;
            if (y < 0) y = 0;
            if (y > OLED_H - 1) y = OLED_H - 1;
            if (y < ymin) ymin = (int16_t)y;
            if (y > ymax) ymax = (int16_t)y;
            y_last = (int16_t)y;
        }
        int16_t lo = ymin, hi = ymax;
        if (prev_y >= 0) {
            if (prev_y < lo) lo = prev_y;
            if (prev_y > hi) hi = prev_y;
        }
        fb_vline((uint8_t)x, lo, hi);
        prev_y = y_last;
    }

    OLED_Refresh();

    printf("Capture OK : ADC brut min=%u max=%u | filtre min=%d max=%d | saut max=%ld a l'ech. %u (col %u) | %s | %lu ms\r\n",
           raw_min, raw_max, vmin, vmax, (long)jump_max, jump_idx, jump_idx / DECIM,
           leads_seen ? "ELECTRODE DECROCHEE" : "electrodes OK",
           (unsigned long)(HAL_GetTick() - t0));
}

static void handle_block(const uint32_t *b)
{
    if (capturing &&
        ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == GPIO_PIN_SET) ||
         (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_SET))) leads_seen = 1;

    for (int i = 0; i < HALF_SIZE; i++)
    {
        uint16_t raw = (uint16_t)(b[i] & 0x0FFF);
        int16_t  v   = hp_step(raw);            // le filtre tourne meme hors capture

        if (capturing && cap_n < CAP_LEN)
        {
            cap[cap_n++] = v;
            if (raw < raw_min) raw_min = raw;
            if (raw > raw_max) raw_max = raw;
        }
    }

    if (capturing && cap_n >= CAP_LEN) { capturing = 0; render_capture(); }
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  printf("=== Demarrage ECG OLED ===\r\n");

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);   // AD8232 SDN = actif

  /* Scan I2C : adresse 0x3C ou 0x3D */
  uint8_t found = 0;
  for (uint8_t a = 0x3C; a <= 0x3D; a++)
  {
      if (HAL_I2C_IsDeviceReady(&hi2c1, a << 1, 2, 10) == HAL_OK)
      {
          oled_addr = a << 1; found = 1;
          printf("OLED trouve: 0x%02X\r\n", a);
          break;
      }
  }
  if (!found) { printf("OLED non trouve : verifier SDA/SCL\r\n"); Error_Handler(); }

  OLED_Init();
  memset(fb, 0, sizeof(fb));
  uint32_t t0 = HAL_GetTick();
  OLED_Refresh();
  printf("Refresh OLED : %lu ms (doit etre < 100 ms)\r\n", (unsigned long)(HAL_GetTick() - t0));

  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) Error_Handler();
  if (HAL_ADC_Start_DMA(&hadc1, adc_buffer, BUFFER_SIZE) != HAL_OK)    Error_Handler();
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)                            Error_Handler();
  printf("Pret : appui sur PA11 = nouvelle capture\r\n");
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  uint8_t btn_prev = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        uint8_t btn = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET);
    if (btn && !btn_prev)
    {
        cap_n = 0; leads_seen = 0; raw_min = 4095; raw_max = 0; capturing = 1;
        dma_half_flag = 0; dma_complete_flag = 0;     // on repart sur un demi-buffer frais
        printf("Capture...\r\n");
        HAL_Delay(30);
    }
    btn_prev = btn;

    if (dma_half_flag)     { dma_half_flag = 0;     handle_block(&adc_buffer[0]); }
    if (dma_complete_flag) { dma_complete_flag = 0; handle_block(&adc_buffer[HALF_SIZE]); }
  }
  /* USER CODE END 3 */
}

/* ======================= Config peripheriques (CubeMX) ======================= */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) Error_Handler();

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

static void MX_ADC1_Init(void)
{
  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) Error_Handler();

  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;     // ~100 kHz ; voir note : passer en 400 kHz dans CubeMX
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) Error_Handler();
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) Error_Handler();
}

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;                         // 80 MHz / 80 / 1000 = 1000 Hz
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) Error_Handler();
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

  /* PC5 / PC6 : LO- / LO+ de l'AD8232 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* PC8 : SDN AD8232 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* PA11 : bouton poussoir (actif bas) */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) dma_half_flag = 1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) dma_complete_flag = 1;
}
/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
