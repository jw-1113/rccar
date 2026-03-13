/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "car.h"
#include "hcsr04_3.h"
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

/* USER CODE BEGIN PV */

volatile uint16_t dist_left = 0;
volatile uint16_t dist_front  = 0;
volatile uint16_t dist_right = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  HCSR04_3_TIM_IC_Callback(htim);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef enum { DIR_F=0, DIR_L, DIR_R }Dir_t;

#define VALID_MIN_CM		3
#define VALID_MAX_CM		300

#define STOP_CM		23		// 전방 이하면 전진 금지
#define SAFE_CM		30		// 전방이 이 이상이면 직진
#define TURN_MS		130		// 좌,우 회전 유지시간(조절)
#define HOLD_MS		90		// 같은 결정을 최소 유지
#define HYST_CM		15		// 이만큼 차이 나야 방향 바꿈

#define FWD_BONUS_CM  15
#define WALL_NEAR_CM   13   // 벽이 이 값보다 가까우면 보정 시작 (15~25 조절)
#define WALL_HYST_CM    5
#define NUDGE_MS  5

static uint32_t holdUntil = 0;
static Dir_t holdDir = DIR_F;

static uint16_t sanitize(uint16_t d){
	if (d < VALID_MIN_CM || d > VALID_MAX_CM) return 0;
	return d;
}

static void AutoDrive_MaxDir(uint16_t dL_raw, uint16_t dF_raw, uint16_t dR_raw)
{
    uint32_t now = HAL_GetTick();

    uint16_t dL = sanitize(dL_raw);
    uint16_t dF = sanitize(dF_raw);
    uint16_t dR = sanitize(dR_raw);

    // 아직 유효시간이면 그대로 유지
    if (now < holdUntil) {
        if (holdDir == DIR_F)      Car_Forward();
        else if (holdDir == DIR_L) Car_Left();
        else                       Car_Right();
        return;
    }

    // 전방 무효(0)면 전진 금지: 좌/우 큰쪽, 둘다 0이면 정지
    if (dF == 0) {
        if (dL == 0 && dR == 0) {
            Car_Stop();
            holdDir = DIR_F;
            holdUntil = now + 80;
            return;
        }
        holdDir = (dR >= dL) ? DIR_R : DIR_L;
        holdUntil = now + TURN_MS;
        if (holdDir == DIR_L) Car_Left();
        else                  Car_Right();
        return;
    }

    // 전방이 너무 가까우면 전진 금지 -> 좌/우 큰쪽으로
    if (dF <= STOP_CM) {
        if (dL == 0 && dR == 0) {
            Car_Stop();
            holdDir = DIR_F;
            holdUntil = now + 80;
            return;
        }
        holdDir = (dR >= dL) ? DIR_R : DIR_L;
        holdUntil = now + TURN_MS;
        if (holdDir == DIR_L) Car_Left();
        else                  Car_Right();
        return;
    }

    // ===== 정상 주행: L/F/R 중 "가장 큰 값" 우선 =====
    Dir_t best = DIR_F;
    uint16_t bestVal = dF;

    // 좌/우는 전방보다 "FWD_BONUS_CM" 이상 더 넓을 때만 꺾기
    if (dL != 0 && dL > (uint16_t)(dF + FWD_BONUS_CM) && dL > bestVal) {
        bestVal = dL; best = DIR_L;
    }
    if (dR != 0 && dR > (uint16_t)(dF + FWD_BONUS_CM) && dR > bestVal) {
        bestVal = dR; best = DIR_R;
    }

    // 히스테리시스: 지금 방향보다 HYST_CM 이상 좋아야 방향 변경
    uint16_t curVal = (holdDir == DIR_F) ? dF : (holdDir == DIR_L) ? dL : dR;
    if (curVal == 0) curVal = 1;

    if (best != holdDir && bestVal < (uint16_t)(curVal + HYST_CM)) {
        best = holdDir;   // 차이 작으면 기존 유지
    }

    holdDir = best;

    // ----- best가 전진이면, 벽 보정(살짝 틀기) -----
    if (holdDir == DIR_F) {

        // (양쪽 다 유효일 때만) 왼쪽 벽 가까우면 오른쪽으로 살짝
        if (dL != 0 && dR != 0 &&
            dL < WALL_NEAR_CM &&
            dR > dL + WALL_HYST_CM)
        {
            holdDir = DIR_R;
            holdUntil = now + NUDGE_MS;
            Car_Right();
            return;
        }

        // (양쪽 다 유효일 때만) 오른쪽 벽 가까우면 왼쪽으로 살짝
        if (dL != 0 && dR != 0 &&
            dR < WALL_NEAR_CM &&
            dL > dR + WALL_HYST_CM)
        {
            holdDir = DIR_L;
            holdUntil = now + NUDGE_MS;
            Car_Left();
            return;
        }

        // 벽 문제 없으면 직진
        holdDir = DIR_F;
        holdUntil = now + HOLD_MS;
        Car_Forward();
        return;
    }

    // ----- 좌/우면 회전 -----
    holdUntil = now + TURN_MS;
    if (holdDir == DIR_L) Car_Left();
    else                  Car_Right();
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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	Car_Init();
	Car_UartStart();
	HAL_TIM_Base_Start(&htim1);
	HCSR04_3_Init(&htim2);

	Car_SetMode(CAR_MODE_STOP);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  static uint32_t last = 0;
	  static uint8_t step = 0;
	  static uint32_t autoTs = 0;

	  if (HAL_GetTick() - last >= 15)   // 60ms마다 하나씩 발사
	  {
	    last = HAL_GetTick();

	    if (step == 0) HCSR04_3_Trigger(US_LEFT);
	    if (step == 1) HCSR04_3_Trigger(US_FRONT);
	    if (step == 2) HCSR04_3_Trigger(US_RIGHT);

	    step = (step + 1) % 3;
	  }

	  dist_left  = HCSR04_3_GetDistance(US_LEFT);
	  dist_front = HCSR04_3_GetDistance(US_FRONT);
	  dist_right = HCSR04_3_GetDistance(US_RIGHT);


	  if (Car_GetMode() == CAR_MODE_AUTO){
		  if (HAL_GetTick() - autoTs >= 60)
		  {
			  autoTs = HAL_GetTick();
			  AutoDrive_MaxDir(dist_left, dist_front, dist_right);
		  }
	  }

	  else if (Car_GetMode() == CAR_MODE_STOP){
		  Car_Stop();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
