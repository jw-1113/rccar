#include "delay.h"




void delay_us(uint16_t us){
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while((__HAL_TIM_GET_COUNTER(&htim1)) < us);
}
