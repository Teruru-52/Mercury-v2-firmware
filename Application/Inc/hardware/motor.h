/**
 * @file motor.h
 * @author Teruru-52
 */

#ifndef HARDWARE_MOTOR_H_
#define HARDWARE_MOTOR_H_

#include "main.h"
#include "hardware/led.h"

namespace hardware
{
    class Motor
    {
    private:
        uint16_t dma_data[3];
        int max_input = 2800 - 1;
        int duty_left;
        int duty_right;
        float battery_voltage;
        float current_left;
        float current_right;
        const int calibration_rounds = 500;
        const float sens_coeff = 1000.0f / 264.0f;
        float offset_current_left = 0.0f;
        float offset_current_right = 0.0f;
        const float zero_current_output = ADC_REF_VOLTAGE * 0.5f;

    public:
        // Motor();

        void StartDMA();
        void CalcOffset();
        void BatteryCheck();
        void UpdateBatteryVoltage();
        void UpdateCurrent();
        int GetDuty(float input_vol);
        void Drive(float v_left, float v_right);
        void Brake();
        void Free();
        void printLog();
    };
} // namespace hardware
#endif //  HARDWARE_MOTOR_H_