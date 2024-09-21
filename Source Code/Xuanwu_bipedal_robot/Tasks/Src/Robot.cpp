#include "Robot.h"
#include "adc.h"
#include "dma.h"

Robot::Robot()
{
    act_left_leg_angles = {0.0f, 0.0f, 0.0f, 0.0f};
    act_right_leg_angles = {0.0f, 0.0f, 0.0f, 0.0f};
    ref_left_leg_angles = {0.0f, 0.0f, 0.0f, 0.0f};
    ref_right_leg_angles = {0.0f, 0.0f, 0.0f, 0.0f};
    ref_left_foot_pos = {-0.095f, 0.0f, -0.559999f };
    ref_right_foot_pos = {0.0f, 0.0f, 0.0f};
    act_left_foot_pos = {0.0f, 0.0f, 0.0f};
    act_right_foot_pos = {0.0f, 0.0f, 0.0f};
}

void Robot::initBatteryADC() const
{
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)battery_adc_val,2);
}

float Robot::getBatteryVoltage()
{
    battery_voltage = (battery_adc_val[0] * 3.3f) / 65535 * 11.0f;
    return battery_voltage;
}
