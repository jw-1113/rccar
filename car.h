#ifndef INC_CAR_H_
#define INC_CAR_H_

#include "main.h"
#include "usart.h"
#include "tim.h"

#include "stm32f1xx_hal.h"
// 초기화(필요시 확장 가능)

typedef enum {
  CAR_MODE_STOP = 0,
  CAR_MODE_MANUAL,
  CAR_MODE_AUTO
} CarMode_t;

void Car_Init(void);

// 동작 함수
void Car_Stop(void);
void Car_Forward(void);
void Car_Backward(void);
void Car_Left(void);
void Car_Right(void);

// UART 수신 시작
void Car_UartStart(void);

// UART 수신 콜백 처리(메인에서 콜백에서 호출해도 되고, car.c에 콜백 자체를 넣어도 됨)
void Car_OnUartByte(uint8_t b);

CarMode_t Car_GetMode(void);
void Car_SetMode(CarMode_t m);

#endif /* INC_CAR_H_ */
