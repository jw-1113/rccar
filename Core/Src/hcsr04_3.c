#include "hcsr04_3.h"
#include "delay.h"   // delay_us() (TIM1 기반)

/* ========= 핀 매핑 (너가 정한 그대로) ========= */
#define TRIG_F_PORT GPIOB
#define TRIG_F_PIN  GPIO_PIN_13

#define TRIG_L_PORT GPIOB
#define TRIG_L_PIN  GPIO_PIN_14

#define TRIG_R_PORT GPIOB
#define TRIG_R_PIN  GPIO_PIN_15

/* ========= 내부 구조 ========= */
typedef struct {
  uint32_t channel;          // TIM_CHANNEL_1/2/3
  GPIO_TypeDef *trigPort;
  uint16_t trigPin;

  uint16_t ic1;
  uint16_t ic2;
  uint16_t echo_us;
  uint16_t distance_cm;

  uint8_t  flag;             // 0: rising 대기, 1: falling 대기
  uint8_t  done;             // 측정 완료
} HCSR04_Sensor_t;

static TIM_HandleTypeDef *g_htim = NULL;
static HCSR04_Sensor_t s[3];

static int sensor_index_from_active_channel(uint32_t activeCh)
{
  if (activeCh == HAL_TIM_ACTIVE_CHANNEL_1) return US_LEFT;
  if (activeCh == HAL_TIM_ACTIVE_CHANNEL_2) return US_FRONT;
  if (activeCh == HAL_TIM_ACTIVE_CHANNEL_3) return US_RIGHT;
  return -1;
}

void HCSR04_3_Init(TIM_HandleTypeDef *htim_ic)
{
  g_htim = htim_ic;

  s[US_LEFT] = (HCSR04_Sensor_t){
    .channel = TIM_CHANNEL_1, .trigPort = TRIG_F_PORT, .trigPin = TRIG_F_PIN
  };
  s[US_FRONT]  = (HCSR04_Sensor_t){
    .channel = TIM_CHANNEL_2, .trigPort = TRIG_L_PORT, .trigPin = TRIG_L_PIN
  };
  s[US_RIGHT] = (HCSR04_Sensor_t){
    .channel = TIM_CHANNEL_3, .trigPort = TRIG_R_PORT, .trigPin = TRIG_R_PIN
  };

  for (int i = 0; i < 3; i++) {
    s[i].distance_cm = 0;
    s[i].flag = 0;
    s[i].done = 0;
    __HAL_TIM_SET_CAPTUREPOLARITY(g_htim, s[i].channel, TIM_INPUTCHANNELPOLARITY_RISING);
  }

  // TIM2 Input Capture 인터럽트 시작 (CH1~CH3)
  HAL_TIM_IC_Start_IT(g_htim, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(g_htim, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(g_htim, TIM_CHANNEL_3);
}

void HCSR04_3_Trigger(US_Id_t id)
{
  if (!g_htim) return;

  // 상태 초기화
  s[id].flag = 0;
  s[id].done = 0;
  __HAL_TIM_SET_CAPTUREPOLARITY(g_htim, s[id].channel, TIM_INPUTCHANNELPOLARITY_RISING);

  // TRIG 10us 펄스 (TIM1 기반 delay_us 사용)
  HAL_GPIO_WritePin(s[id].trigPort, s[id].trigPin, GPIO_PIN_RESET);
  delay_us(2);
  HAL_GPIO_WritePin(s[id].trigPort, s[id].trigPin, GPIO_PIN_SET);
  delay_us(10);
  HAL_GPIO_WritePin(s[id].trigPort, s[id].trigPin, GPIO_PIN_RESET);
}

uint16_t HCSR04_3_GetDistance(US_Id_t id)
{
  return s[id].distance_cm;
}

uint8_t HCSR04_3_IsDone(US_Id_t id)
{
  return s[id].done;
}

void HCSR04_3_TIM_IC_Callback(TIM_HandleTypeDef *htim)
{
  if (!g_htim) return;
  if (htim->Instance != g_htim->Instance) return;

  int idx = sensor_index_from_active_channel(htim->Channel);
  if (idx < 0) return;

  HCSR04_Sensor_t *p = &s[idx];

  if (p->flag == 0)
  {
    // Rising edge 시간 저장
    p->ic1 = HAL_TIM_ReadCapturedValue(htim, p->channel);
    p->flag = 1;
    __HAL_TIM_SET_CAPTUREPOLARITY(htim, p->channel, TIM_INPUTCHANNELPOLARITY_FALLING);
  }
  else
  {
    // Falling edge 시간 저장
    p->ic2 = HAL_TIM_ReadCapturedValue(htim, p->channel);

    uint16_t dt;
    if (p->ic2 >= p->ic1) dt = (uint16_t)(p->ic2 - p->ic1);
    else                  dt = (uint16_t)((0xFFFF - p->ic1) + p->ic2 + 1);

    p->echo_us = dt;
    p->distance_cm = (uint16_t)(p->echo_us / 58);  // HC-SR04 근사(cm)

    p->flag = 0;
    p->done = 1;
    __HAL_TIM_SET_CAPTUREPOLARITY(htim, p->channel, TIM_INPUTCHANNELPOLARITY_RISING);
  }
}
