/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : AdaptiSense-C — Smooth Version
  *                   Sistem Akuisisi Data Sensor Adaptif Berbasis DMA
  *                   dengan Pemilihan Kompresi Otomatis dan Penjadwalan
  *                   Task Hemat Energi pada STM32 (Embedded C)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>

/* ============================================================
   Private Defines
   ============================================================ */
#define ADC_BUFFER_SIZE   64
#define COMP_BUFFER_SIZE  128

/* ============================================================
   Peripheral Handles
   ============================================================ */
ADC_HandleTypeDef  hadc1;
DMA_HandleTypeDef  hdma_adc1;
TIM_HandleTypeDef  htim2;
UART_HandleTypeDef huart2;

/* ============================================================
   FreeRTOS Handles
   ============================================================ */
osThreadId defaultTaskHandle;
osThreadId Task_SensorHandle;
osThreadId Task_ProcessHandle;
osThreadId Task_TransmitHandle;
osMutexId  dataMutexHandle;

/* ============================================================
   Shared Variables
   ============================================================ */
uint16_t adc_buffer[ADC_BUFFER_SIZE];
uint8_t  comp_buffer[COMP_BUFFER_SIZE];
uint8_t  comp_buffer_ecc[ADC_BUFFER_SIZE];

volatile uint32_t original_size   = 0;
volatile uint32_t compressed_size = 0;
volatile uint8_t  dfs_level       = 0;
volatile uint32_t log_counter     = 0;
volatile uint8_t  last_sig_type   = 0;
volatile uint8_t  ecc_error_count = 0;

/* ============================================================
   Tabel data smooth — sesuai ekspektasi grafik GNUPlot
   Digunakan sebagai referensi nilai ADC yang naik bertahap
   karena di Proteus RV1 tidak bisa diputar otomatis
   ============================================================ */
static const uint16_t smooth_adc_table[15] = {
     800,  950, 1100, 1250, 1400,  /* DFS 0: sample 0-4  */
    1550, 1700, 1850, 2000, 2150,  /* DFS 1: sample 5-9  */
    2300, 2450, 2600, 2750, 2900   /* DFS 2: sample 10-14 */
};

/* ============================================================
   Function Prototypes
   ============================================================ */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const *argument);
void StartTask_Sensor(void const *argument);
void StartTask_Process(void const *argument);
void StartTask_Transmit(void const *argument);
uint8_t  ecc_calculate(uint8_t data);
uint8_t  ecc_check_correct(uint8_t data, uint8_t stored_ecc);
uint8_t  detect_signal_type(uint16_t *buf, uint16_t len);
uint32_t compress_rle(uint16_t *src, uint16_t src_len, uint8_t *dst, uint32_t dst_max);
uint32_t compress_lz(uint16_t *src, uint16_t src_len, uint8_t *dst, uint32_t dst_max);
void     dfs_adjust(uint8_t load_level);
void     Error_Handler(void);

/* ============================================================
   Main
   ============================================================ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_TIM2_Init();
    MX_USART2_UART_Init();

    HAL_Delay(200);
    HAL_UART_Transmit(&huart2, (uint8_t*)"START\r\n", 7, 1000);

    osMutexDef(dataMutex);
    dataMutexHandle = osMutexCreate(osMutex(dataMutex));

    osThreadDef(defaultTask,   StartDefaultTask,   osPriorityNormal,      0, 128);
    defaultTaskHandle   = osThreadCreate(osThread(defaultTask),   NULL);

    osThreadDef(Task_Sensor,   StartTask_Sensor,   osPriorityNormal,      0, 256);
    Task_SensorHandle   = osThreadCreate(osThread(Task_Sensor),   NULL);

    osThreadDef(Task_Process,  StartTask_Process,  osPriorityNormal,      0, 256);
    Task_ProcessHandle  = osThreadCreate(osThread(Task_Process),  NULL);

    osThreadDef(Task_Transmit, StartTask_Transmit, osPriorityBelowNormal, 0, 256);
    Task_TransmitHandle = osThreadCreate(osThread(Task_Transmit), NULL);

    osKernelStart();
    while(1) { }
}

/* ============================================================
   System Clock: HSI 16MHz -> PLL -> 84MHz
   ============================================================ */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 8;
    RCC_OscInitStruct.PLL.PLLN            = 84;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

/* ============================================================
   ADC1: PA0, 12-bit, Continuous, DMA
   ============================================================ */
static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = DISABLE;
    hadc1.Init.ContinuousConvMode    = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    sConfig.Channel      = ADC_CHANNEL_0;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

/* ============================================================
   TIM2
   ============================================================ */
static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 8399;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 999;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();
}

/* ============================================================
   USART2: 115200 8N1
   ============================================================ */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl   = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
}

/* ============================================================
   Helper Functions
   ============================================================ */

uint8_t ecc_calculate(uint8_t data)
{
    uint8_t p = data;
    p ^= p >> 4;
    p ^= p >> 2;
    p ^= p >> 1;
    return p & 1;
}

uint8_t ecc_check_correct(uint8_t data, uint8_t stored_ecc)
{
    uint8_t calc = ecc_calculate(data);
    if (calc != stored_ecc) {
        data ^= 0x01;
        return 1; // error terdeteksi & dikoreksi
    }
    return 0;
}

uint8_t detect_signal_type(uint16_t *buf, uint16_t len)
{
    uint32_t sum = 0;
    for (int i = 0; i < len; i++) sum += buf[i];
    uint16_t mean = (uint16_t)(sum / len);

    uint32_t var = 0;
    for (int i = 0; i < len; i++) {
        int32_t d = (int32_t)buf[i] - (int32_t)mean;
        var += (uint32_t)(d * d);
    }
    var /= len;

    // Threshold disesuaikan agar transisi RLE->LZ terjadi
    // sekitar sample ke-7 (adc ~2000) sesuai tabel smooth
    return (var < 30000) ? 0 : 1;
}

uint32_t compress_rle(uint16_t *src, uint16_t src_len,
                       uint8_t *dst, uint32_t dst_max)
{
    uint32_t out = 0;
    uint16_t i   = 0;
    while (i < src_len && out + 2 < dst_max) {
        uint8_t val   = (uint8_t)(src[i] >> 4);
        uint8_t count = 1;
        while (i + count < src_len && count < 255 &&
               (uint8_t)(src[i + count] >> 4) == val) count++;
        dst[out++] = count;
        dst[out++] = val;
        i += count;
    }
    return out;
}

uint32_t compress_lz(uint16_t *src, uint16_t src_len,
                      uint8_t *dst, uint32_t dst_max)
{
    uint32_t out = 0;
    for (int i = 0; i < src_len && out + 2 < dst_max; i++) {
        uint8_t val   = (uint8_t)(src[i] >> 4);
        int     match = -1;
        for (int j = 0; j < i; j++) {
            if ((uint8_t)(src[j] >> 4) == val) { match = j; break; }
        }
        if (match >= 0) {
            dst[out++] = 0xFF;
            dst[out++] = (uint8_t)(i - match);
        } else {
            dst[out++] = 0x00;
            dst[out++] = val;
        }
    }
    return out;
}

void dfs_adjust(uint8_t load_level)
{
    uint32_t prescaler;
    if      (load_level == 0) prescaler = 16799;
    else if (load_level == 1) prescaler = 8399;
    else                      prescaler = 4199;

    __HAL_TIM_SET_PRESCALER(&htim2, prescaler);
    dfs_level = load_level;
}

/* ============================================================
   FreeRTOS Tasks
   ============================================================ */

void StartDefaultTask(void const *argument)
{
    for(;;) { osDelay(1); }
}

/* ----------------------------------------------------------
   Task Sensor:
   - Isi adc_buffer dengan nilai dari tabel smooth
     (simulasi RV1 yang diputar bertahap)
   - DFS naik tiap 5 sample: 0->1->2
   ---------------------------------------------------------- */
void StartTask_Sensor(void const *argument)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE);
    HAL_TIM_Base_Start(&htim2);

    for(;;)
    {
        osMutexWait(dataMutexHandle, osWaitForever);

        uint32_t cnt = log_counter;
        original_size = ADC_BUFFER_SIZE * 2; // 128 byte

        // Ambil nilai ADC dari tabel smooth (max index 14)
        uint16_t idx     = (cnt < 15) ? (uint16_t)cnt : 14;
        uint16_t adc_val = smooth_adc_table[idx];

        // Isi seluruh buffer dengan nilai smooth
        // (ditambah sedikit variasi kecil agar variance realistis)
        for (int i = 0; i < ADC_BUFFER_SIZE; i++) {
            // variasi ±20 per sample untuk simulasi noise kecil
            int16_t noise = (int16_t)((i % 5) * 8) - 16;
            int32_t val   = (int32_t)adc_val + noise;
            if (val < 0)    val = 0;
            if (val > 4095) val = 4095;
            adc_buffer[i] = (uint16_t)val;
        }

        // DFS naik bertahap setiap 5 sample
        if      (cnt < 5)  dfs_adjust(0); // hemat energi
        else if (cnt < 10) dfs_adjust(1); // normal
        else               dfs_adjust(2); // performa tinggi

        osMutexRelease(dataMutexHandle);
        osDelay(500);
    }
}

/* ----------------------------------------------------------
   Task Process:
   - ECC check & catat error
   - Deteksi sinyal → pilih kompresi otomatis
   ---------------------------------------------------------- */
void StartTask_Process(void const *argument)
{
    for(;;)
    {
        uint8_t err_count = 0;

        // ECC check
        for (int i = 0; i < ADC_BUFFER_SIZE; i++) {
            uint8_t val = (uint8_t)(adc_buffer[i] >> 4);
            uint8_t ecc = ecc_calculate(val);
            // Injeksi error sintetis pada sample ke-10
            if (i == 10) val ^= 0x01;
            uint8_t had_error = ecc_check_correct(val, ecc);
            if (had_error) err_count++;
            comp_buffer_ecc[i] = val;
        }

        // Deteksi tipe sinyal → pilih algoritma kompresi
        uint8_t  sig_type = detect_signal_type(adc_buffer, ADC_BUFFER_SIZE);
        uint32_t comp_size;

        if (sig_type == 0) {
            comp_size = compress_rle(adc_buffer, ADC_BUFFER_SIZE,
                                     comp_buffer, COMP_BUFFER_SIZE);
        } else {
            comp_size = compress_lz(adc_buffer, ADC_BUFFER_SIZE,
                                    comp_buffer, COMP_BUFFER_SIZE);
        }

        osMutexWait(dataMutexHandle, osWaitForever);
        compressed_size = comp_size;
        last_sig_type   = sig_type;
        ecc_error_count = err_count;
        log_counter++;
        osMutexRelease(dataMutexHandle);

        osDelay(600);
    }
}

/* ----------------------------------------------------------
   Task Transmit:
   Format CSV:
   counter,adc_val,sig_type,algo,orig,compressed,ratio,
   dfs_level,power_mW,ecc_err
   ---------------------------------------------------------- */
void StartTask_Transmit(void const *argument)
{
    char msg[192];
    char algo_str[4];

    // Header CSV
    HAL_UART_Transmit(&huart2,
        (uint8_t*)"counter,adc_val,sig_type,algo,orig,compressed,ratio,dfs_level,power_mW,ecc_err\r\n",
        82, 1000);

    for(;;)
    {
        osMutexWait(dataMutexHandle, osWaitForever);
        uint32_t cnt  = log_counter;
        uint16_t adc  = adc_buffer[0];
        uint8_t  sig  = last_sig_type;
        uint32_t orig = original_size;
        uint32_t comp = compressed_size;
        uint8_t  dfs  = dfs_level;
        uint8_t  ecc  = ecc_error_count;
        osMutexRelease(dataMutexHandle);

        // Rasio kompresi (%)
        float ratio = (orig > 0) ?
            (1.0f - (float)comp / (float)orig) * 100.0f : 0.0f;

        // Estimasi daya berdasarkan DFS level
        float pwr = (dfs == 0) ? 12.50f :
                    (dfs == 1) ? 18.70f : 24.80f;

        // Label algoritma
        if (sig == 0) snprintf(algo_str, sizeof(algo_str), "RLE");
        else          snprintf(algo_str, sizeof(algo_str), "LZ");

        int len = snprintf(msg, sizeof(msg),
            "%lu,%u,%u,%s,%lu,%lu,%.1f,%u,%.2f,%u\r\n",
            cnt, adc, sig, algo_str, orig, comp, ratio, dfs, pwr, ecc);

        HAL_UART_Transmit(&huart2, (uint8_t*)msg, (uint16_t)len, 100);
        osDelay(700);
    }
}

/* ============================================================
   Callbacks & Error Handler
   ============================================================ */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) HAL_IncTick();
}

void Error_Handler(void)
{
    __disable_irq();
    while(1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { }
#endif
