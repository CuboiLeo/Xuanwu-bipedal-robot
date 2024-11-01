#include "Onboard_Buzzer.h"
#include "tim.h"
#include "cmsis_os.h"

void Buzzer::Init() {
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
}

void Buzzer::Beep() {
    __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2, 1000);
    osDelay(200);
    __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2, 0);
}
