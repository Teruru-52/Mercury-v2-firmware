/**
 * @file motor.cpp
 * @author Teruru-52
 */

#include "hardware/motor.h"

namespace hardware
{
    void
    Motor::StartDMA()
    {
        HAL_ADC_Start_DMA(&hadc2, (uint32_t *)dma_data, 3);
    }

    void Motor::BatteryCheck()
    {
        UpdateBatteryVoltage();
        if (battery_voltage > 8.2)
            led.OnFrontLeft3();
        if (battery_voltage > 7.8)
            led.OnFrontLeft2();
        if (battery_voltage > 7.6)
            led.OnFrontLeft1();
        if (battery_voltage > 7.4)
            led.OnFrontRight1();
        if (battery_voltage > 7.2)
            led.OnFrontRight2();
        if (battery_voltage > 7.0)
            led.OnFrontRight3();
        printf("battery: %.2f [V]\n", battery_voltage);
    }

    void Motor::UpdateBatteryVoltage()
    {
        battery_voltage = static_cast<float>(dma_data[2]) * ADC_REF_VOLTAGE / ADC_RESOLUTION * 3.0f;
    }

    void Motor::CalcOffset()
    {
        Write_GPIO(LED_RED, GPIO_PIN_SET);
        float current_left_sum = 0;
        float current_right_sum = 0;
        for (int i = 0; i < calibration_rounds; i++)
        {
            UpdateCurrent();
            current_left_sum += current_left;
            current_right_sum += current_right;
            HAL_Delay(1);
        }
        offset_current_left = current_left_sum / static_cast<float>(calibration_rounds);
        offset_current_right = current_right_sum / static_cast<float>(calibration_rounds);

        printf("offset_current_left: %.3f, offset_current_right: %.3f\n", offset_current_left, offset_current_right);
        Write_GPIO(LED_RED, GPIO_PIN_RESET);
    }

    void Motor::UpdateCurrent()
    {
        float amp_voltage_left = static_cast<float>(dma_data[0]) * ADC_REF_VOLTAGE / ADC_RESOLUTION;
        float amp_voltage_right = static_cast<float>(dma_data[1]) * ADC_REF_VOLTAGE / ADC_RESOLUTION;
        current_left = (amp_voltage_left - zero_current_output) * sens_coeff - offset_current_left;
        current_right = (amp_voltage_right - zero_current_output) * sens_coeff - offset_current_right;
    }

    int Motor::GetDuty(float input_voltage)
    {
        return (int)((float)max_input * input_voltage / battery_voltage);
    }

    void Motor::Drive(float v_left, float v_right)
    {
        UpdateBatteryVoltage();
        UpdateCurrent();

        duty_left = GetDuty(v_left);
        duty_right = GetDuty(v_right);

        if (duty_left > max_input)
            duty_left = max_input;
        else if (duty_left < -max_input)
            duty_left = -max_input;

        if (duty_right > max_input)
            duty_right = max_input;
        else if (duty_right < -max_input)
            duty_right = -max_input;

        if (duty_left > 0)
        {
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, max_input - duty_left);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, max_input);
        }
        else
        {
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, max_input);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, max_input + duty_left);
        }

        if (duty_right > 0)
        {
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, max_input);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, max_input - duty_right);
        }
        else
        {
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, max_input + duty_right);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, max_input);
        }
    }

    void Motor::Brake()
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, max_input);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, max_input);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, max_input);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, max_input);
    }

    void Motor::Free()
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
    }

    void Motor::printLog()
    {
        printf("battery_voltage: %.3f, current_left: %.3f, current_right: %.3f\n", battery_voltage, current_left, current_right);
    }
}