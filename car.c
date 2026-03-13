#include "car.h"

// 너 코드에서 쓰던 수신 바이트
static uint8_t rx;
static volatile CarMode_t g_mode = CAR_MODE_STOP;
// ====== 내부 함수 ======
static void Car_SetAllOff(void)
{
  TIM3->CCR1 = 0;
  TIM3->CCR2 = 0;
  TIM3->CCR3 = 0;
  TIM3->CCR4 = 0;
}

// ====== 외부 함수 ======
void Car_Init(void)
{
  // PWM Start는 main에서 하는 걸 추천(초기화 흐름 명확)
  Car_Stop();
}

void Car_Stop(void)
{
  Car_SetAllOff();
}

void Car_Forward(void)
{
  Car_Stop();
  TIM3->CCR2 = 349;
  TIM3->CCR4 = 349;
}

void Car_Backward(void)
{
  Car_Stop();
  TIM3->CCR1 = 349;
  TIM3->CCR3 = 349;
}

void Car_Left(void)
{
  Car_Stop();
  TIM3->CCR2 = 299;
//  TIM3->CCR3 = 199;
}

void Car_Right(void)
{
  Car_Stop();
//  TIM3->CCR2 = 199;
  TIM3->CCR4 = 299;
}

void Car_UartStart(void)
{
  // USART1 기준 (너 코드 그대로)
  HAL_UART_Receive_IT(&huart1, &rx, 1);
}

// 수신된 1바이트를 동작으로 변환
void Car_OnUartByte(uint8_t b)
{
	if (b == 'T') { Car_SetMode(CAR_MODE_MANUAL); return; }
	if (b == 'X') { Car_SetMode(CAR_MODE_AUTO); return; }

	if (g_mode == CAR_MODE_STOP) return;

	if (g_mode == CAR_MODE_AUTO) return;


	if      (b == 'F') Car_Forward();
	else if (b == 'B') Car_Backward();
	else if (b == 'L') Car_Left();
	else if (b == 'R') Car_Right();
	else if (b == '0') Car_Stop();
}

CarMode_t Car_GetMode(void) { return g_mode; }

void Car_SetMode(CarMode_t m){
	g_mode = m;
	Car_Stop();
}

// ====== UART 콜백을 car.c에 “직접” 넣는 방식 ======
// main.c에 콜백을 두고 Car_OnUartByte만 호출해도 되고,
// 아래처럼 car.c에 콜백을 넣어도 됨(둘 중 하나만 선택!)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    Car_OnUartByte(rx);
    HAL_UART_Receive_IT(&huart1, &rx, 1);
  }
}
