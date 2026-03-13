#ifndef INC_HCSR04_3_H_
#define INC_HCSR04_3_H_

#include "main.h"
#include "tim.h"
#include <stdint.h>

typedef enum {
  US_LEFT = 0,
  US_FRONT  = 1,
  US_RIGHT = 2
} US_Id_t;

// TIM2 핸들로 초기화 (CH1~CH3 IC Start)
void HCSR04_3_Init(TIM_HandleTypeDef *htim_ic);

// 센서 트리거(10us 펄스)
void HCSR04_3_Trigger(US_Id_t id);

// 마지막으로 계산된 거리(cm)
uint16_t HCSR04_3_GetDistance(US_Id_t id);

// 마지막 트리거 후 측정 완료 여부
uint8_t HCSR04_3_IsDone(US_Id_t id);

// HAL_TIM_IC_CaptureCallback에서 호출해줘야 함
void HCSR04_3_TIM_IC_Callback(TIM_HandleTypeDef *htim);

#endif /* INC_HCSR04_3_H_ */
