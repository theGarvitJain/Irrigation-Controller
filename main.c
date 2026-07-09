/* USER CODE BEGIN Header */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t pipe[4];
uint8_t pwm[4];
uint8_t pipe_num;
uint8_t start_time[4];
uint8_t stop_time[4];
uint8_t pwm_num;
uint16_t clock_hours = 0;
uint16_t clock_secs = 0;
uint8_t last_hour = 0;
uint16_t min_distance = 100;
uint16_t max_distance = 250;
uint8_t cmd_dist = 0x55;
uint8_t us100_Rx_flag = 00;
uint8_t us100_buffer[2] = {0};
volatile uint16_t distance = 0;
volatile uint16_t depth = 100;
volatile uint8_t rcv_intpt_flag = 0;
uint16_t rpm_tick_count = 0;
uint8_t buffer[64] = {0};
uint8_t double_byte[2];
int empty_zone = 0;
int single_digit = 1;
volatile uint8_t hcsr07_Rx_flag = 0;
volatile uint8_t first_edge = 0;
volatile uint16_t time_edge1 = 0;
volatile uint16_t time_edge2 = 0;
/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
UART_HandleTypeDef huart2;
/* USER CODE BEGIN PV */
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HCSR07_TRIG_PULSE(void) {
 HAL_GPIO_WritePin(HCSR07_TRIG_GPIO_Port, HCSR07_TRIG_Pin, GPIO_PIN_SET);
 for(volatile int i = 0; i < 20; i++) {}
 HAL_GPIO_WritePin(HCSR07_TRIG_GPIO_Port, HCSR07_TRIG_Pin, GPIO_PIN_RESET);
}
int get_active_pipeline() {
 for (int i = 0; i < 4; i++) {
 if (start_time[i] <= clock_hours && clock_hours < stop_time[i]) {
 return i;
 }
 }
 return -1;
}
void DIGITS_Display() {
 uint8_t DIGIT_A = depth / 10;
 uint8_t DIGIT_B = depth % 10;
 uint8_t DIGITA_VAL = 0x0F & DIGIT_A;
 uint8_t DIGITB_VAL = 0x0F & DIGIT_B;
 int Abit0 = (DIGITA_VAL) & 1;
 int Abit1 = (DIGITA_VAL >> 1) & 1;
 int Abit2 = (DIGITA_VAL >> 2) & 1;
 int Abit3 = (DIGITA_VAL >> 3) & 1;
 int Bbit0 = (DIGITB_VAL) & 1;
 int Bbit1 = (DIGITB_VAL >> 1) & 1;
 int Bbit2 = (DIGITB_VAL >> 2) & 1;
 int Bbit3 = (DIGITB_VAL >> 3) & 1;
 HAL_GPIO_WritePin(GPIOB, DIGIT_A0_Pin, Abit0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
 HAL_GPIO_WritePin(GPIOB, DIGIT_A1_Pin, Abit1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
 HAL_GPIO_WritePin(GPIOB, DIGIT_A2_Pin, Abit2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
 HAL_GPIO_WritePin(GPIOB, DIGIT_A3_Pin, Abit3 ? GPIO_PIN_SET : GPIO_PIN_RESET);
 HAL_GPIO_WritePin(GPIOC, DIGIT_B0_Pin, Bbit0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
 HAL_GPIO_WritePin(GPIOC, DIGIT_B1_Pin, Bbit1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
 HAL_GPIO_WritePin(GPIOC, DIGIT_B2_Pin, Bbit2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
 HAL_GPIO_WritePin(GPIOC, DIGIT_B3_Pin, Bbit3 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
void ADC_Select_CH(int CH) {
ADC_ChannelConfTypeDef sConfig = {0};
switch(CH)
{
case 0:
sConfig.Channel = ADC_CHANNEL_0;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 1:
sConfig.Channel = ADC_CHANNEL_1;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 2:
sConfig.Channel = ADC_CHANNEL_2;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 3:
sConfig.Channel = ADC_CHANNEL_3;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 4:
sConfig.Channel = ADC_CHANNEL_4;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 5:
sConfig.Channel = ADC_CHANNEL_5;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 6:
sConfig.Channel = ADC_CHANNEL_6;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 7:
sConfig.Channel = ADC_CHANNEL_7;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 8:
sConfig.Channel = ADC_CHANNEL_8;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 9:
sConfig.Channel = ADC_CHANNEL_9;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 10:
sConfig.Channel = ADC_CHANNEL_10;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 11:
sConfig.Channel = ADC_CHANNEL_11;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 12:
sConfig.Channel = ADC_CHANNEL_12;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 13:
sConfig.Channel = ADC_CHANNEL_13;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 14:
sConfig.Channel = ADC_CHANNEL_14;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 15:
sConfig.Channel = ADC_CHANNEL_15;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
}
}
void user_input () {
sprintf((char*)buffer, "\r\nSETUP Mode");
HAL_UART_Transmit(&huart2, buffer, strlen((char *)buffer),
HAL_MAX_DELAY);
memset(buffer, 0, sizeof(buffer));
for (int i = 0; i < 4; i++) {
single_digit = 1;
rcv_intpt_flag = 00;
HAL_UART_Receive_IT(&huart2, double_byte, 1);
sprintf((char*)buffer, "\r\n\nEnter Pipeline Choice (0 = INLET,
1-3 = ZONES): ");
HAL_UART_Transmit(&huart2, buffer, strlen((char *)buffer),
HAL_MAX_DELAY);
while(rcv_intpt_flag == (00)) {};
pipe[i] = atoi((char *)&double_byte);
memset(buffer, 0, sizeof(buffer));
memset(double_byte, 0, sizeof(double_byte));
rcv_intpt_flag = 00;
HAL_UART_Receive_IT(&huart2, double_byte, 1);
sprintf((char*)buffer, "\r\nEnter Pump PWM (0 = MANUAL, 1-3 =
FIXED): ");
HAL_UART_Transmit(&huart2, buffer, strlen((char *)buffer),
HAL_MAX_DELAY);
while(rcv_intpt_flag == (00)) {};
pwm[i] = atoi((char *)&double_byte);
memset(buffer, 0, sizeof(buffer));
memset(double_byte, 0, sizeof(double_byte));
}
for (int i = 0; i < 4; i ++) {
single_digit = 0;
rcv_intpt_flag = 00;
HAL_UART_Receive_IT(&huart2, double_byte, 2);
sprintf((char*)buffer, "\r\n\nEnter Start Hour (00 - 23): ");
HAL_UART_Transmit(&huart2, buffer, strlen((char *)buffer),
HAL_MAX_DELAY);
while(rcv_intpt_flag == (00)) {};
start_time[i] = atoi((char *)&double_byte);
memset(buffer, 0, sizeof(buffer));
memset(double_byte, 0, sizeof(double_byte));
rcv_intpt_flag = 00;
HAL_UART_Receive_IT(&huart2, double_byte, 2);
sprintf((char*)buffer, "\r\nEnter Stop Hour (00 - 23): ");
HAL_UART_Transmit(&huart2, buffer, strlen((char *)buffer),
HAL_MAX_DELAY);
rcv_intpt_flag = 00;
while(rcv_intpt_flag == (00)) {};
stop_time[i] = atoi((char *)&double_byte);
memset(buffer, 0, sizeof(buffer));
memset(double_byte, 0, sizeof(double_byte));
}
rcv_intpt_flag = 00;
while(rcv_intpt_flag) {};
sprintf((char*)buffer, "\r\n\nPrinting SETUP Parameters");
HAL_UART_Transmit(&huart2, buffer, strlen((char *)buffer),
HAL_MAX_DELAY);
memset(buffer, 0, sizeof(buffer));
sprintf((char*)buffer, "\r\nCurrent Wall Clock Hour: 00");
HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer),
HAL_MAX_DELAY);
memset(buffer, 0, sizeof(buffer));
for (int i = 0; i < 4; i++) {
sprintf((char *)buffer, "\r\nPipeline: %u, Pump PWM: %u,
Start Hour: %u, Stop Hour: %u", pipe[i], pwm[i], start_time[i], stop_time[i]);
HAL_UART_Transmit(&huart2, buffer, strlen((char *)buffer),
HAL_MAX_DELAY);
memset(buffer, 0, sizeof(buffer));
}
sprintf((char*)buffer, "\r\n\nSETUP is done. Press Blue Button
for RUN Mode");
HAL_UART_Transmit(&huart2, buffer, strlen((char *)buffer),
HAL_MAX_DELAY);
memset(buffer, 0, sizeof(buffer));
}
uint32_t get_potentio_num() {
 ADC_Select_CH(9);
 HAL_ADC_Start(&hadc1);
 HAL_ADC_PollForConversion(&hadc1, 1000);
 uint32_t ADC_CH9 = HAL_ADC_GetValue(&hadc1);
 HAL_ADC_Stop(&hadc1);
 uint32_t percentage = (ADC_CH9 * 100) / 255;
 return percentage;
}
void servo_num(uint16_t pipe_num) {
if (pipe_num == 0) {
TIM2->CCR1 = 500;
} else if (pipe_num == 1) {
TIM2->CCR1 = 800;
} else if (pipe_num == 2) {
TIM2->CCR1 = 1100;
} else if (pipe_num == 3) {
TIM2->CCR1 = 1400;
}
}
void DC_motor_speed(uint16_t pipe_num, uint16_t pwm_num) {
 if(pipe_num == 0) {
 TIM3->CCR3 = 0;
 if (pwm_num == 0) {
 uint32_t pot_percent = get_potentio_num();
 TIM3->CCR1 = (pot_percent * 2000) / 100;
 } else if (pwm_num == 1) {
 TIM3->CCR1 = (uint32_t)(0.7 * 2000);
 } else if (pwm_num == 2) {
 TIM3->CCR1 = (uint32_t)(0.85 * 2000);
 } else if (pwm_num == 3) {
 TIM3->CCR1 = (uint32_t)(0.99 * 2000);
 }
 } else {
 TIM3->CCR1 = 0;
 if (pwm_num == 0) {
 uint32_t pot_percent = get_potentio_num();
 TIM3->CCR3 = (pot_percent * 2000) / 100;
 } else if (pwm_num == 1) {
 TIM3->CCR3 = (uint32_t)(0.7 * 2000);
 } else if (pwm_num == 2) {
 TIM3->CCR3 = (uint32_t)(0.85 * 2000);
 } else if (pwm_num == 3) {
 TIM3->CCR3 = (uint32_t)(0.99 * 2000);
 }
 }
 HAL_Delay(100);
}
void DC_motor_reset() {
TIM3->CCR1 = 0;
TIM3->CCR3 = 0;
}
void water_level(uint16_t current_pipe, uint16_t current_pwm) {
 if (current_pipe == 0) {
 // INLET: replenish water by 10-15% per hour depending on PWM
 uint16_t replenish_rate;
 if (current_pwm == 0) {
 uint32_t pot = get_potentio_num();
 replenish_rate = (pot * 15) / 100;
 if (replenish_rate < 5) replenish_rate = 5;
 } else if (current_pwm == 1) {
 replenish_rate = 10;
 } else if (current_pwm == 2) {
 replenish_rate = 12;
 } else {
 replenish_rate = 15;
 }
 depth += replenish_rate;
 if (depth > 99) depth = 99;
 } else {
 // ZONES: deplete water by 8-12% per hour depending on PWM
 uint16_t depletion_rate;
 if (current_pwm == 1) {
 depletion_rate = 8;
 } else if (current_pwm == 2) {
 depletion_rate = 10;
 } else {
 depletion_rate = 12;
 }
 if (depth > depletion_rate) {
 depth -= depletion_rate;
 } else {
 depth = 0;
 }
 }
}
uint16_t get_rpm() {
 uint16_t revolutions = rpm_tick_count / 4;
 uint16_t rpm = (revolutions * 60) / 12;
 rpm_tick_count = 0;
 return rpm;
}
void run_output() {
 uint16_t rpm = get_rpm();
 uint16_t report_hour = (clock_hours > 0) ? (clock_hours - 1) : 0;
 if(empty_zone) {
 sprintf((char *)buffer, "\r\n%02d : : : %d : %d",
 report_hour, rpm, depth);
 } else {
 sprintf((char *)buffer, "\r\n%02d : %d : %d : %d : %d",
 report_hour, pipe_num, pwm_num, rpm, depth);
 }
 HAL_UART_Transmit(&huart2, buffer, strlen((char *)buffer), HAL_MAX_DELAY);
 memset(buffer, 0, sizeof(buffer));
}
void set_led(uint16_t pipe_num) {
 HAL_GPIO_WritePin(GPIOC, LED_Blue_Pin, GPIO_PIN_RESET);
 HAL_GPIO_WritePin(GPIOC, LED_Green_Pin, GPIO_PIN_RESET);
 HAL_GPIO_WritePin(GPIOB, LED_Red_Pin, GPIO_PIN_RESET);
 if (pipe_num == 0) {
 HAL_GPIO_WritePin(GPIOB, LED_Red_Pin, GPIO_PIN_SET);
 HAL_GPIO_WritePin(GPIOC, LED_Blue_Pin, GPIO_PIN_SET);
 } else if (pipe_num == 1) {
 HAL_GPIO_WritePin(GPIOB, LED_Red_Pin, GPIO_PIN_SET);
 } else if (pipe_num == 2) {
 HAL_GPIO_WritePin(GPIOC, LED_Green_Pin, GPIO_PIN_SET);
 } else if (pipe_num == 3) {
 HAL_GPIO_WritePin(GPIOC, LED_Blue_Pin, GPIO_PIN_SET);
 } else if (pipe_num == 4) {
 // Flashing white for empty reservoir
 HAL_GPIO_WritePin(GPIOB, LED_Red_Pin, GPIO_PIN_SET);
 HAL_GPIO_WritePin(GPIOC, LED_Blue_Pin, GPIO_PIN_SET);
 HAL_GPIO_WritePin(GPIOC, LED_Green_Pin, GPIO_PIN_SET);
 }
}
void reset_led() {
 HAL_GPIO_WritePin(GPIOC, LED_Blue_Pin, GPIO_PIN_RESET);
 HAL_GPIO_WritePin(GPIOC, LED_Green_Pin, GPIO_PIN_RESET);
 HAL_GPIO_WritePin(GPIOB, LED_Red_Pin, GPIO_PIN_RESET);
}
void flash_white_led() {
 for(int i = 0; i < 10; i++) {
 set_led(4);
 HAL_Delay(200);
 reset_led();
 HAL_Delay(200);
 }
}
/* USER CODE END 0 */
/**
 * @brief The application entry point.
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
 MX_USART2_UART_Init();
 MX_TIM3_Init();
 MX_TIM4_Init();
 MX_TIM5_Init();
 MX_TIM2_Init();
 MX_ADC1_Init();
 /* USER CODE BEGIN 2 */
 HAL_TIM_Base_Start(&htim4);
 HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
 HAL_TIM_Base_Init(&htim2);
 HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
 TIM2->PSC = 16;
 TIM2->ARR = 20000;
 TIM2->CCR1 = 500;
 HAL_TIM_Base_Init(&htim3);
 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
 TIM3->PSC = 16;
 TIM3->ARR = 2000;
 TIM3->CCR1 = 0;
 TIM3->CCR3 = 0;
 HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
 user_input();
 while(HAL_GPIO_ReadPin(GPIOC, USER_Button_Pin) == GPIO_PIN_SET) {
HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
HAL_Delay(100);
HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
HAL_Delay(100);
 }
 HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
 sprintf((char *)buffer, "\r\n\nRUN MODE");
 HAL_UART_Transmit(&huart2, buffer, strlen((char *)buffer), HAL_MAX_DELAY);
 memset(buffer, 0, sizeof(buffer));
 sprintf((char *)buffer, "\r\nHOUR : PIPE : PWM : RPM : DEPTH");
 HAL_UART_Transmit(&huart2, buffer, strlen((char*) buffer), HAL_MAX_DELAY);
 memset(buffer, 0, sizeof(buffer));
 sprintf((char *)buffer, "\r\n---------------------------------------------");
 HAL_UART_Transmit(&huart2, buffer, strlen((char *)buffer), HAL_MAX_DELAY);
 memset(buffer, 0, sizeof(buffer));
 clock_hours = 0;
 clock_secs = 0;
 last_hour = 0;
 HAL_TIM_Base_Start_IT(&htim5);
 /* USER CODE END 2 */
 /* Infinite loop */
 /* USER CODE BEGIN WHILE */
 while(1) {
 int active_idx = get_active_pipeline();
 if (active_idx >= 0) {
 empty_zone = 0;
 pipe_num = pipe[active_idx];
 pwm_num = pwm[active_idx];
 servo_num(pipe_num);
 set_led(pipe_num);
 if (pwm_num == 0) {
 uint32_t hour_start = clock_hours;
 while (clock_hours == hour_start) {
 DC_motor_speed(pipe_num, pwm_num);
HAL_Delay(100);
 }
 } else {
 DC_motor_speed(pipe_num, pwm_num);
 while (clock_hours == last_hour) {
 HAL_Delay(10);
 }
 }
 } else {
 empty_zone = 1;
 DC_motor_reset();
 reset_led();
 while (clock_hours == last_hour) {
 HAL_Delay(10);
 }
 }
 last_hour = clock_hours;
 water_level(pipe_num, pwm_num);
 if (depth == 0) {
 DC_motor_reset();
 TIM2->CCR1 = 0;
 HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
 // Set depth to 0 and display on timer board
 depth = 0;
 DIGITS_Display();
 sprintf((char *)buffer, "\r\n\nRESERVOIR IS EMPTY");
 HAL_UART_Transmit(&huart2, buffer, strlen((char *)buffer),
HAL_MAX_DELAY);
 memset(buffer, 0, sizeof(buffer));
 flash_white_led();
 break;
 }
 DIGITS_Display();
 run_output();
 if (clock_hours >= 24) {
 DC_motor_reset();
 reset_led();
 HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
 TIM2->CCR1 = 0;
 sprintf((char *)buffer, "\r\n\nIrrigation complete");
 HAL_UART_Transmit(&huart2, buffer, strlen((char *)buffer), HAL_MAX_DELAY);
 memset(buffer, 0, sizeof(buffer));
 break;
 }
 }
 /* USER CODE END WHILE */
 /* USER CODE BEGIN 3 */
HAL_GPIO_WritePin(GPIOC, LED_Blue_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOB, LED_Red_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOC, LED_Green_Pin, GPIO_PIN_RESET);
HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
HAL_TIM_Base_Stop_IT(&htim5);
return 0;
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
 __HAL_RCC_PWR_CLK_ENABLE();
 __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
 /** Initializes the RCC Oscillators according to the specified parameters
 * in the RCC_OscInitTypeDef structure.
 */
 RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
 RCC_OscInitStruct.HSIState = RCC_HSI_ON;
 RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
 RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
 if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
 {
 Error_Handler();
 }
 /** Initializes the CPU, AHB and APB buses clocks
 */
 RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
 |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
 RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
 RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
 RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
 RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
 if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
 /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and
number of conversion)
 */
 hadc1.Instance = ADC1;
 hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
 hadc1.Init.Resolution = ADC_RESOLUTION_8B;
 hadc1.Init.ScanConvMode = DISABLE;
 hadc1.Init.ContinuousConvMode = DISABLE;
 hadc1.Init.DiscontinuousConvMode = DISABLE;
 hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
 hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
 hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
 hadc1.Init.NbrOfConversion = 1;
 hadc1.Init.DMAContinuousRequests = DISABLE;
 hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
 if (HAL_ADC_Init(&hadc1) != HAL_OK)
 {
 Error_Handler();
 }
 /** Configure for the selected ADC regular channel its corresponding rank in the
sequencer and its sample time.
 */
 sConfig.Channel = ADC_CHANNEL_9;
 sConfig.Rank = 1;
 sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
 if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
 Error_Handler();
 }
 /* USER CODE BEGIN ADC1_Init 2 */
 /* USER CODE END ADC1_Init 2 */
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
 TIM_MasterConfigTypeDef sMasterConfig = {0};
 TIM_OC_InitTypeDef sConfigOC = {0};
 /* USER CODE BEGIN TIM2_Init 1 */
 /* USER CODE END TIM2_Init 1 */
 htim2.Instance = TIM2;
 htim2.Init.Prescaler = 54;
 htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
 htim2.Init.Period = 1000-1;
 htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
 htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
 /* USER CODE BEGIN TIM2_Init 2 */
 /* USER CODE END TIM2_Init 2 */
 HAL_TIM_MspPostInit(&htim2);
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
 TIM_OC_InitTypeDef sConfigOC = {0};
 /* USER CODE BEGIN TIM3_Init 1 */
 /* USER CODE END TIM3_Init 1 */
 htim3.Instance = TIM3;
 htim3.Init.Prescaler = 16-1;
 htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
 htim3.Init.Period = 2000-1;
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
 if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
 {
 Error_Handler();
 }
 sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
 sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
 if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
 {
 Error_Handler();
 }
 sConfigOC.OCMode = TIM_OCMODE_PWM1;
 sConfigOC.Pulse = 0;
 sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
 sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
 if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
 {
 Error_Handler();
 }
 if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
 {
 Error_Handler();
 }
 /* USER CODE BEGIN TIM3_Init 2 */
 /* USER CODE END TIM3_Init 2 */
 HAL_TIM_MspPostInit(&htim3);
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
 TIM_IC_InitTypeDef sConfigIC = {0};
 /* USER CODE BEGIN TIM4_Init 1 */
 /* USER CODE END TIM4_Init 1 */
 htim4.Instance = TIM4;
 htim4.Init.Prescaler = 16-1;
 htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
 htim4.Init.Period = 65535-1;
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
 if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
 {
 Error_Handler();
 }
 sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
 sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
 if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
 {
 Error_Handler();
 }
 sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
 sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
 sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
 sConfigIC.ICFilter = 8;
 if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
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
 TIM_ClockConfigTypeDef sClockSourceConfig = {0};
 TIM_MasterConfigTypeDef sMasterConfig = {0};
 /* USER CODE BEGIN TIM5_Init 1 */
 /* USER CODE END TIM5_Init 1 */
 htim5.Instance = TIM5;
 htim5.Init.Prescaler = 53-1;
 htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
 htim5.Init.Period = 59939;
 htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
 htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
 if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
 {
 Error_Handler();
 }
 sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
 if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
 {
 Error_Handler();
 }
 sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
 sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
 if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
 {
 Error_Handler();
 }
 /* USER CODE BEGIN TIM5_Init 2 */
 /* USER CODE END TIM5_Init 2 */
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
 HAL_GPIO_WritePin(GPIOC, DIGIT_B3_Pin|DIGIT_A3_Pin|LED_Blue_Pin|LED_Green_Pin
 |DIGIT_B0_Pin|DIGIT_B1_Pin|DIGIT_B2_Pin, GPIO_PIN_RESET);
 /*Configure GPIO pin Output Level */
 HAL_GPIO_WritePin(GPIOA, DIGIT_A1_Pin|LD2_Pin|HCSR07_TRIG_Pin, GPIO_PIN_RESET);
 /*Configure GPIO pin Output Level */
 HAL_GPIO_WritePin(GPIOB, LED_Red_Pin|DIGIT_A0_Pin|DIGIT_A2_Pin, GPIO_PIN_RESET);
 /*Configure GPIO pin : USER_Button_Pin */
 GPIO_InitStruct.Pin = USER_Button_Pin;
 GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 HAL_GPIO_Init(USER_Button_GPIO_Port, &GPIO_InitStruct);
 /*Configure GPIO pins : DIGIT_B3_Pin DIGIT_A3_Pin LED_Blue_Pin LED_Green_Pin
 DIGIT_B0_Pin DIGIT_B1_Pin DIGIT_B2_Pin */
 GPIO_InitStruct.Pin = DIGIT_B3_Pin|DIGIT_A3_Pin|LED_Blue_Pin|LED_Green_Pin
 |DIGIT_B0_Pin|DIGIT_B1_Pin|DIGIT_B2_Pin;
 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
 /*Configure GPIO pins : DIGIT_A1_Pin LD2_Pin HCSR07_TRIG_Pin */
 GPIO_InitStruct.Pin = DIGIT_A1_Pin|LD2_Pin|HCSR07_TRIG_Pin;
 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
 /*Configure GPIO pin : RPM_TICK_Pin */
 GPIO_InitStruct.Pin = RPM_TICK_Pin;
 GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 HAL_GPIO_Init(RPM_TICK_GPIO_Port, &GPIO_InitStruct);
 /*Configure GPIO pins : LED_Red_Pin DIGIT_A0_Pin DIGIT_A2_Pin */
 GPIO_InitStruct.Pin = LED_Red_Pin|DIGIT_A0_Pin|DIGIT_A2_Pin;
 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
 /* EXTI interrupt init*/
 HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
 HAL_NVIC_EnableIRQ(EXTI2_IRQn);
 /* USER CODE BEGIN MX_GPIO_Init_2 */
 /* USER CODE END MX_GPIO_Init_2 */
}
/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
 if (htim->Instance == TIM4) {
 if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
 if (first_edge == 0) {
 time_edge1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
 first_edge = 1;
 } else {
 time_edge2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
 uint16_t pulse_width;
 if (time_edge2 >= time_edge1) {
 pulse_width = time_edge2 - time_edge1;
 } else {
 pulse_width = (65536 - time_edge1) + time_edge2;
 }
 if (pulse_width >= 150 && pulse_width <= 25000) {
 hcsr07_Rx_flag = 1;
 }
 first_edge = 0;
 __HAL_TIM_SET_COUNTER(htim, 0);
 }
 }
 }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
 if (huart->Instance == USART2) {
 if (us100_Rx_flag == 0 && rcv_intpt_flag == 0) {
 if (single_digit) {
 HAL_UART_Transmit(&huart2, double_byte, 1, HAL_MAX_DELAY);
 } else {
 HAL_UART_Transmit(&huart2, double_byte, 2, HAL_MAX_DELAY);
 }
 rcv_intpt_flag = 1;
 } else if (us100_Rx_flag == 0) {
 us100_Rx_flag = 1;
 }
 }
}
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin) {
if (GPIO_Pin == RPM_TICK_Pin) {
rpm_tick_count++;
}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
 if (htim->Instance == TIM5) {
 clock_secs++;
 clock_hours = clock_secs/60;
 if (clock_hours > 24) {
 clock_hours = 24;
 }
 }
}
/* USER CODE END 4 */
/**
 * @brief This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
 /* USER CODE BEGIN Error_Handler_Debug */
 /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
 /* USER CODE BEGIN 6 */
 /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */