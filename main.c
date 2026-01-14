/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */



/* ====== KONFIGŪRACIJA ====== */
#define SIM800_UART                 huart1

#define DEVICE_ID     "bokstas-001"
#define PHONE_NUMBER  "+37062735795"   // <-- pakeisk
#define SMS_PERIOD_MS 60000UL

/* ====== SHT40 + PCA9548A KONFIGŪRACIJA ====== */
#define SHT40_ADDR                  (0x44 << 1)      // 7-bit address shifted for HAL
#define SHT40_MEASURE_HIGH_PREC     0xFD
#define PCA9548A_ADDR               (0x70 << 1)

#define TEMP_RIBA                   25.0f  //25.0f
#define DREGMES_RIBA                14.5f  //14.5f

#define JUTIKLIU_KIEKIS             6


#define CURRENT_LIMIT_MA  910.0f
#define SHUNT_RESISTOR    0.1f
#define ADC_REF_VOLTAGE   3.3f

/* “MC” formulės konstantos */
static const float A_const = 0.000043295f;
static const float B_const = 2.11190f;
static const float C_const = 41.565f;

/* Matavimų buferiai */
static float temp[JUTIKLIU_KIEKIS];
static float hum_mc[JUTIKLIU_KIEKIS];
static uint8_t valid[JUTIKLIU_KIEKIS];

static uint32_t last_sms = 0;
static uint32_t last_measure = 0;

static int scd30_status = 0;   // 0=init, 1=started, -1=no dev, -2=start fail, -3=read fail
static uint32_t scd30_last_read = 0;

static uint32_t motor_cycle_timer = 0;
static uint8_t motor_active = 0;
static uint32_t last_current_measure = 0;

float current = 0;


uint32_t motor_start_time = 0;
#define MOTOR_STARTUP_IGNORE_MS 1500  // 1.5 sekundės ignoruojame šuolį


#define SCD30_ADDR                  (0x61 << 1)

/* SCD30 reikšmės */
static float scd30_co2_ppm = 0.0f;
static float scd30_temp_c  = 0.0f;
static float scd30_rh_pct  = 0.0f;

static uint8_t scd30_started = 0;
static uint32_t scd30_next_poll = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */

static void SIM800_SendCmd(const char *cmd);
static void SIM800_SendSMS(const char *number, const char *text);

static HAL_StatusTypeDef PCA9548A_SelectChannel(uint8_t channel);
static HAL_StatusTypeDef SHT40_Read(uint8_t channel, float *temperatura, float *dregme_mc);

/* SCD30 */
static uint8_t scd30_crc8(const uint8_t *data, int len);
static HAL_StatusTypeDef SCD30_StartContinuousMeasurement(void);
static HAL_StatusTypeDef SCD30_GetDataReady(uint8_t *ready);
static HAL_StatusTypeDef SCD30_ReadMeasurement(float *co2_ppm, float *temp_c, float *rh_pct);
static void SCD30_Task(void);

/* Formatavimas su kableliu */
static void fmt_float_comma(char *out, size_t out_sz, float v, int decimals);

float Measure_Current(void);
void SIM800_Init_Receive(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float Measure_Current(void) {
    uint32_t total_raw = 0;
    int samples = 10; // nuskaitome 10 kartų

    for(int i = 0; i < samples; i++) {
        HAL_ADC_Start(&hadc);
        HAL_ADC_PollForConversion(&hadc, 2);
        total_raw += HAL_ADC_GetValue(&hadc);
    }

    float avg_raw = (float)total_raw / samples;
    float voltage = (avg_raw * ADC_REF_VOLTAGE) / 4095.0f;
    return (voltage / SHUNT_RESISTOR) * 1000.0f;
}

// Funkcija išvalyti SIM800 buferį ir paruošti priėmimui
void SIM800_Init_Receive(void) {
    SIM800_SendCmd("AT+CNMI=2,2,0,0,0\r\n"); // Tiesioginis SMS rodymas UART'e
}

/* ---------------- Pagalbinė funkcija: float -> "12,34" ---------------- */
static void fmt_float_comma(char *out, size_t out_sz, float v, int decimals)
{
    if (!out || out_sz == 0) return;

    char fmt[8];
    // pvz. "%.2f"
    snprintf(fmt, sizeof(fmt), "%%.%df", decimals);

    snprintf(out, out_sz, fmt, (double)v);

    // pakeičiam '.' į ','
    for (size_t i = 0; i < out_sz && out[i] != '\0'; i++)
    {
        if (out[i] == '.')
            out[i] = ',';
    }
}

/* ---------------- SIM800 ---------------- */
static void SIM800_SendCmd(const char *cmd)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)cmd, (uint16_t)strlen(cmd), 1000);
    HAL_Delay(300);
}

static void SIM800_SendSMS(const char *number, const char *text)
{
    char cmd[64];

    SIM800_SendCmd("AT\r\n");
    SIM800_SendCmd("ATE0\r\n");
    SIM800_SendCmd("AT+CMGF=1\r\n");   // SMS TEXT MODE

    snprintf(cmd, sizeof(cmd), "AT+CMGS=\"%s\"\r\n", number);
    SIM800_SendCmd(cmd);

    HAL_UART_Transmit(&huart1, (uint8_t*)text, (uint16_t)strlen(text), 3000);

    uint8_t ctrlZ = 26;
    HAL_UART_Transmit(&huart1, &ctrlZ, 1, 1000);

    HAL_Delay(5000); // palaukiam išsiuntimo
}

/* ---------------- PCA9548A ---------------- */
static HAL_StatusTypeDef PCA9548A_SelectChannel(uint8_t channel)
{
    uint8_t data = (uint8_t)(1U << channel);
    return HAL_I2C_Master_Transmit(&hi2c1, PCA9548A_ADDR, &data, 1, 20);
}

/* ---------------- SHT40 ---------------- */
static HAL_StatusTypeDef SHT40_Read(uint8_t channel, float *temperatura, float *dregme_mc)
{
    uint8_t cmd = SHT40_MEASURE_HIGH_PREC;
    uint8_t data[6];
    HAL_StatusTypeDef status;

    status = PCA9548A_SelectChannel(channel);
    if (status != HAL_OK)
    {
        *temperatura = 0.0f;
        *dregme_mc   = 0.0f;
        return status;
    }

    status = HAL_I2C_IsDeviceReady(&hi2c1, SHT40_ADDR, 2, 20);
    if (status != HAL_OK)
    {
        *temperatura = 0.0f;
        *dregme_mc   = 0.0f;
        return status;
    }

    status = HAL_I2C_Master_Transmit(&hi2c1, SHT40_ADDR, &cmd, 1, 50);
    if (status != HAL_OK)
    {
        *temperatura = 0.0f;
        *dregme_mc   = 0.0f;
        return status;
    }

    HAL_Delay(10);

    status = HAL_I2C_Master_Receive(&hi2c1, SHT40_ADDR, data, 6, 50);
    if (status != HAL_OK)
    {
        *temperatura = 0.0f;
        *dregme_mc   = 0.0f;
        return status;
    }

    uint16_t rawT  = (uint16_t)((data[0] << 8) | data[1]);
    uint16_t rawRH = (uint16_t)((data[3] << 8) | data[4]);

    *temperatura = -45.0f + (175.0f * (float)rawT  / 65535.0f);      // °C
    float rh_pct = -6.0f  + (125.0f * (float)rawRH / 65535.0f);      // %RH

    /* MC = [ -ln(1 - RH) / (A * (T + C)) ]^(1/B) */
    float RH = rh_pct / 100.0f;
    if (RH <= 0.0f) RH = 0.0001f;
    if (RH >= 1.0f) RH = 0.9999f;

    float skaitiklis   = -logf(1.0f - RH);
    float vardiklis    = A_const * (*temperatura + C_const);
    float skliaustuose = skaitiklis / vardiklis;

    if (skliaustuose <= 0.0f)
    {
        *dregme_mc = 0.0f;
        return HAL_OK;
    }

    float MC = powf(skliaustuose, 1.0f / B_const);
    float MC_wb = (MC / (100.0f + MC)) * 100.0f;

    *dregme_mc = MC_wb;

    return HAL_OK;
}

/* ================= SCD30 ================= */
static uint8_t scd30_crc8(const uint8_t *data, int len)
{
    // CRC-8 polynomial 0x31, init 0xFF (Sensirion)
    uint8_t crc = 0xFF;
    for (int i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
        {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static HAL_StatusTypeDef SCD30_StartContinuousMeasurement(void)
{
    // Command: 0x0010, arg: 0x0000 (ambient pressure disabled)
    uint8_t cmd[5];
    cmd[0] = 0x00; cmd[1] = 0x10;
    cmd[2] = 0x00; cmd[3] = 0x00;
    cmd[4] = scd30_crc8(&cmd[2], 2);

    return HAL_I2C_Master_Transmit(&hi2c2, SCD30_ADDR, cmd, 5, 200);
}

static inline uint8_t SCD30_DataReady_GPIO(void)
{
    return (HAL_GPIO_ReadPin(SCD30_RDY_GPIO_Port, SCD30_RDY_Pin) == GPIO_PIN_SET);
}

static HAL_StatusTypeDef SCD30_ReadMeasurement(float *co2_ppm, float *temp_c, float *rh_pct)
{
    // Command: 0x0300, read 3 floats => 18 bytes (2 bytes + CRC per word)
    uint8_t cmd[2] = {0x03, 0x00};
    uint8_t rx[18];

    if (HAL_I2C_Master_Transmit(&hi2c2, SCD30_ADDR, cmd, 2, 200) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(2);

    if (HAL_I2C_Master_Receive(&hi2c2, SCD30_ADDR, rx, 18, 200) != HAL_OK)
        return HAL_ERROR;

    for (int i = 0; i < 18; i += 3)
    {
        if (scd30_crc8(&rx[i], 2) != rx[i + 2])
            return HAL_ERROR;
    }

    uint32_t co2_u =
        ((uint32_t)rx[0] << 24) | ((uint32_t)rx[1] << 16) |
        ((uint32_t)rx[3] <<  8) | ((uint32_t)rx[4] <<  0);

    uint32_t t_u =
        ((uint32_t)rx[6] << 24) | ((uint32_t)rx[7] << 16) |
        ((uint32_t)rx[9] <<  8) | ((uint32_t)rx[10] << 0);

    uint32_t rh_u =
        ((uint32_t)rx[12] << 24) | ((uint32_t)rx[13] << 16) |
        ((uint32_t)rx[15] <<  8) | ((uint32_t)rx[16] << 0);

    memcpy(co2_ppm, &co2_u, 4);
    memcpy(temp_c,  &t_u,   4);
    memcpy(rh_pct,  &rh_u,  4);

    return HAL_OK;
}

static void SCD30_Task(void)
{
	uint32_t now = HAL_GetTick();

	    /* 1) Startuojam vieną kartą */
	    if (!scd30_started)
	    {
	        if (HAL_I2C_IsDeviceReady(&hi2c2, SCD30_ADDR, 3, 100) != HAL_OK)
	        {
	            scd30_status = -1; // nemato per I2C
	            return;
	        }

	        if (SCD30_StartContinuousMeasurement() != HAL_OK)
	        {
	            scd30_status = -2; // start fail
	            return;
	        }

	        scd30_started   = 1;
	        scd30_status    = 1;
	        scd30_next_poll = now + 2500;   // laukiam pirmo matavimo
	        return;
	    }

	    /* 2) Neskaitom per dažnai */
	    if ((int32_t)(now - scd30_next_poll) < 0)
	        return;

	    /* 3) Tikrinam RDY PINĄ */
	    if (!SCD30_DataReady_GPIO())
	    {
	        // dar nėra naujų duomenų
	        return;
	    }

	    /* 4) Bandom skaityti */
	    if (SCD30_ReadMeasurement(&scd30_co2_ppm,
	                              &scd30_temp_c,
	                              &scd30_rh_pct) != HAL_OK)
	    {
	        scd30_status = -3; // read / CRC fail
	        scd30_next_poll = now + 1000; // bandysim dar kartą po 1s
	        return;
	    }

	    /* 5) Sėkmingas skaitymas */
	    scd30_status    = 1;
	    scd30_last_read = now;
	    scd30_next_poll = now + 2000; // SCD30 tipinis intervalas
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(Pilnumo_variklis_GPIO_Port, Pilnumo_variklis_Pin, GPIO_PIN_SET);

  HAL_Delay(4000); // SIM800 boot

    last_sms = HAL_GetTick();
    last_measure = HAL_GetTick();
    scd30_next_poll = HAL_GetTick() + 1000;

    SIM800_Init_Receive();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
     {


    	 uint32_t now = HAL_GetTick();

    	      /* SCD30 atnaujinam kas ~1s (CO2 + T + RH) */
    	      SCD30_Task();



    	      // 1. Tikriname, ar laikas paleisti variklį (kas 60000 ms)
    	          if (!motor_active && (now - motor_cycle_timer >= 60000UL))
    	          {
    	              motor_cycle_timer = now;
    	              motor_start_time = now;
    	              motor_active = 1;
    	              // ĮJUNGTI variklį (RESET = ON jūsų schemoje)
    	              HAL_GPIO_WritePin(Pilnumo_variklis_GPIO_Port, Pilnumo_variklis_Pin, GPIO_PIN_RESET);
    	          }

    	          // 2. Jei variklis veikia, tikriname srovę ir veikimo laiką
    	          if (motor_active)
    	          {
    	        	  // A. PALEIDIMO LOGIKA: Kas 60 sekundžių įjungiam variklį
    	        	  if (!motor_active && (now - motor_cycle_timer >= 60000UL)) {
    	        	      motor_active = 1;
    	        	      motor_start_time = now; // Užfiksuojam TIKSLŲ įjungimo laiką

    	        	      // ĮJUNGIAM: RESET = ON (pagal jūsų schemą)
    	        	      HAL_GPIO_WritePin(Pilnumo_variklis_GPIO_Port, Pilnumo_variklis_Pin, GPIO_PIN_RESET);
    	        	  }

    	        	  // B. VEIKIMO LOGIKA: Tikrinam srovę ir laiko limitą
    	        	  if (motor_active) {

    	        	      // APSAUGA: Srovę matuojame TIK jei praėjo daugiau nei 1.5s (Startup Ignore)
    	        	      // Tai apsaugo nuo srovės šuolio fiksavimo paleidimo metu
    	        	      if (now - motor_start_time > MOTOR_STARTUP_IGNORE_MS) {

    	        	          // Matuojame srovę kas 200ms, kad neapkrautume procesoriaus
    	        	          if (now - last_current_measure >= 200) {
    	        	              last_current_measure = now;
    	        	              float current = Measure_Current();

    	        	              if (current > CURRENT_LIMIT_MA) {
    	        	                  // STABDOME: viršyta srovė (užsipildė)
    	        	                  motor_active = 0;
    	        	                  HAL_GPIO_WritePin(Pilnumo_variklis_GPIO_Port, Pilnumo_variklis_Pin, GPIO_PIN_SET);
    	        	                  SIM800_SendSMS(PHONE_NUMBER, "FULL");
    	        	                  motor_cycle_timer = now; // Pradedam laukti naujos minutės nuo dabar
    	        	              }
    	        	          }
    	        	      }

    	        	      // C. LAIKO LIMITAS: Jei srovė nebuvo viršyta, po 5s variklį išjungiam
    	        	      if (motor_active && (now - motor_start_time >= 5000UL)) {
    	        	          motor_active = 0;
    	        	          // IŠJUNGIAM: SET = OFF
    	        	          HAL_GPIO_WritePin(Pilnumo_variklis_GPIO_Port, Pilnumo_variklis_Pin, GPIO_PIN_SET);
    	        	          motor_cycle_timer = now; // Ciklas baigtas, laukiam minutės
    	        	      }
    	        	  }
    	          }


    	      /* ---- 1) SHT40 nuskaitymai kas 1s ---- */
    	      if (now - last_measure >= 1000UL)
    	      {
    	          last_measure = now;

    	          if (HAL_I2C_IsDeviceReady(&hi2c1, PCA9548A_ADDR, 2, 20) != HAL_OK)
    	          {
    	              for (uint8_t i = 0; i < JUTIKLIU_KIEKIS; i++)
    	              {
    	                  valid[i]  = 0;
    	                  temp[i]   = 0.0f;
    	                  hum_mc[i] = 0.0f;
    	              }
    	          }
    	          else
    	          {
    	              for (uint8_t i = 0; i < JUTIKLIU_KIEKIS; i++)
    	              {
    	                  HAL_StatusTypeDef st = SHT40_Read(i, &temp[i], &hum_mc[i]);
    	                  valid[i] = (st == HAL_OK) ? 1U : 0U;
    	                  if (!valid[i])
    	                  {
    	                      temp[i]   = 0.0f;
    	                      hum_mc[i] = 0.0f;
    	                  }
    	              }
    	          }

    	          /* Ventiliatoriaus logika pagal ribas (SHT40) */
    	          uint8_t overLimit = 0;
    	          for (uint8_t i = 0; i < JUTIKLIU_KIEKIS; i++)
    	          {
    	              if (valid[i])
    	              {
    	                  if (temp[i] > TEMP_RIBA || hum_mc[i] > DREGMES_RIBA)
    	                  {
    	                      overLimit = 1;
    	                      break;
    	                  }
    	              }
    	          }

    	          if (overLimit)
    	          {
    	              HAL_GPIO_WritePin(Vedinimo_ventiliatorius_GPIO_Port,
    	                                Vedinimo_ventiliatorius_Pin,
    	                                GPIO_PIN_RESET);
    	          }
    	          else
    	          {
    	              HAL_GPIO_WritePin(Vedinimo_ventiliatorius_GPIO_Port,
    	                                Vedinimo_ventiliatorius_Pin,
    	                                GPIO_PIN_SET);
    	          }
    	      }

    	      /* ---- 2) SMS kas SMS_PERIOD_MS ---- */
    	      if (now - last_sms >= SMS_PERIOD_MS)
    	      {
    	          last_sms = now;

    	          /* Paruošiam su kableliu */
    	          char T1[16],T2[16],T3[16],T4[16],T5[16],T6[16];
    	          char D1[16],D2[16],D3[16],D4[16],D5[16],D6[16];

    	          float tval[JUTIKLIU_KIEKIS];
    	          float dval[JUTIKLIU_KIEKIS];

    	          for (uint8_t i = 0; i < JUTIKLIU_KIEKIS; i++)
    	          {
    	              if (valid[i])
    	              {
    	                  tval[i] = temp[i];
    	                  dval[i] = hum_mc[i];
    	              }
    	              else
    	              {
    	                  tval[i] = 0.0f;
    	                  dval[i] = 0.0f;
    	              }
    	          }

    	          fmt_float_comma(T1, sizeof(T1), tval[0], 2);
    	          fmt_float_comma(T2, sizeof(T2), tval[1], 2);
    	          fmt_float_comma(T3, sizeof(T3), tval[2], 2);
    	          fmt_float_comma(T4, sizeof(T4), tval[3], 2);
    	          fmt_float_comma(T5, sizeof(T5), tval[4], 2);
    	          fmt_float_comma(T6, sizeof(T6), tval[5], 2);

    	          fmt_float_comma(D1, sizeof(D1), dval[0], 2);
    	          fmt_float_comma(D2, sizeof(D2), dval[1], 2);
    	          fmt_float_comma(D3, sizeof(D3), dval[2], 2);
    	          fmt_float_comma(D4, sizeof(D4), dval[3], 2);
    	          fmt_float_comma(D5, sizeof(D5), dval[4], 2);
    	          fmt_float_comma(D6, sizeof(D6), dval[5], 2);

    	          /* Tall iš SCD30 temperatūros (su kableliu) */
    	          char Tall[16];
    	          if (scd30_started)
    	              fmt_float_comma(Tall, sizeof(Tall), scd30_temp_c, 2);
    	          else
    	              strncpy(Tall, "0,00", sizeof(Tall));

    	          /* CO2 iš SCD30 (ppm, sveikas) */
    	          int CO2 = (scd30_started && scd30_co2_ppm > 0.0f) ? (int)(scd30_co2_ppm + 0.5f) : 0;

    	          int FAN = (HAL_GPIO_ReadPin(Vedinimo_ventiliatorius_GPIO_Port,
    	                                      Vedinimo_ventiliatorius_Pin) == GPIO_PIN_RESET) ? 1 : 0;

    	          /* CODE: 0=OK, 1=viršytos ribos, 2=nėra multiplekserio */
    	          int CODE = 0;

    	          if (HAL_I2C_IsDeviceReady(&hi2c1, PCA9548A_ADDR, 2, 20) != HAL_OK)
    	          {
    	              CODE = 2;
    	          }
    	          else
    	          {
    	              for (uint8_t i = 0; i < JUTIKLIU_KIEKIS; i++)
    	              {
    	                  if (valid[i] && (temp[i] > TEMP_RIBA || hum_mc[i] > DREGMES_RIBA))
    	                  {
    	                      CODE = 1;
    	                      break;
    	                  }
    	              }
    	          }

    	          int LVL = 0;

    	          char sms[420];
    	          snprintf(sms, sizeof(sms),
    	                   "ID=%s;"
    	                   "T1=%s;T2=%s;T3=%s;T4=%s;T5=%s;T6=%s;"
    	                   "D1=%s;D2=%s;D3=%s;D4=%s;D5=%s;D6=%s;"
    	                   "Tall=%s;CO2=%d;LVL=%d;FAN=%d;CODE=%d",
    	                   DEVICE_ID,
    	                   T1,T2,T3,T4,T5,T6,
    	                   D1,D2,D3,D4,D5,D6,
    	                   Tall, CO2, LVL, FAN, CODE);

    	          SIM800_SendSMS(PHONE_NUMBER, sms);
    	      }

    	      HAL_Delay(50);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
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
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = ENABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
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
  hi2c1.Init.Timing = 0x00000608;
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
  hi2c2.Init.Timing = 0x00000608;
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
  huart1.Init.BaudRate = 9600;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Vedinimo_ventiliatorius_Pin|Pilnumo_variklis_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SCD30_RDY_Pin */
  GPIO_InitStruct.Pin = SCD30_RDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SCD30_RDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Vedinimo_ventiliatorius_Pin Pilnumo_variklis_Pin */
  GPIO_InitStruct.Pin = Vedinimo_ventiliatorius_Pin|Pilnumo_variklis_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  __HAL_RCC_SYSCFG_CLK_ENABLE();   // Needed for EXTI on L0 series

    HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);  // PC13 is EXTI13 → EXTI4_15 IRQ
    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


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
#ifdef USE_FULL_ASSERT
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
