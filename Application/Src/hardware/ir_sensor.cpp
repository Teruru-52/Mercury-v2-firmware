/**
 * @file ir_sensor.cpp
 * @author Teruru-52
 */

#include "hardware/ir_sensor.h"

namespace hardware
{
    IRsensor::IRsensor(float ir_start_base, IR_Base *ir_is_wall)
        : ir_start_base(ir_start_base),
          ir_is_wall(ir_is_wall) {}

    void IRsensor::OnLed()
    {
        // __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 70);
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 70);
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 70);
    }

    void IRsensor::OffLed()
    {
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
    }

    void IRsensor::UI_led_onoff(const IR_Value &ir_value)
    {
        if (ir_value.fl1 > ir_is_wall->fl1)
            led.OnFrontLeft1();
        else
            led.OffFrontLeft1();

        if (ir_value.fl2 > ir_is_wall->fl2)
            led.OnFrontLeft2();
        else
            led.OffFrontLeft2();

        if (ir_value.fl3 > ir_is_wall->fl3)
            led.OnFrontLeft3();
        else
            led.OffFrontLeft3();

        if (ir_value.fr1 > ir_is_wall->fr1)
            led.OnFrontRight1();
        else
            led.OffFrontRight1();

        if (ir_value.fr2 > ir_is_wall->fr2)
            led.OnFrontRight2();
        else
            led.OffFrontRight2();

        if (ir_value.fr3 > ir_is_wall->fr3)
            led.OnFrontRight3();
        else
            led.OffFrontRight3();
    }

    void IRsensor::UI_led_off()
    {
        led.OffFrontLeft1();
        led.OffFrontLeft2();
        led.OffFrontLeft3();
        led.OffFrontRight1();
        led.OffFrontRight2();
        led.OffFrontRight3();
    }

    void IRsensor::PrintWalldata(const IR_Value &ir_value)
    {
        if (ir_value.fl3 > ir_is_wall->fl3)
            printf("1");
        else
            printf("0");
        if (ir_value.fl2 > ir_is_wall->fl2)
            printf("1");
        else
            printf("0");
        if (ir_value.fl1 > ir_is_wall->fl1)
            printf("1");
        else
            printf("0");
        if (ir_value.fr1 > ir_is_wall->fr1)
            printf("1");
        else
            printf("0");
        if (ir_value.fr2 > ir_is_wall->fr2)
            printf("1");
        else
            printf("0");
        if (ir_value.fr3 > ir_is_wall->fr3)
            printf("1");
        else
            printf("0");
    }

    void IRsensor::StartDMA()
    {
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)dma_ir, 6);
    }

    void IRsensor::UpdateLeftValue()
    {
        if (dma_ir[2] > max_fl1)
            max_fl1 = dma_ir[2];
        if (dma_ir[1] > max_fl2)
            max_fl2 = dma_ir[1];
        if (dma_ir[0] > max_fl3)
            max_fl3 = dma_ir[0];
    }

    void IRsensor::UpdateRightValue()
    {
        if (dma_ir[5] > max_fr1)
            max_fr1 = dma_ir[5];
        if (dma_ir[4] > max_fr2)
            max_fr2 = dma_ir[4];
        if (dma_ir[3] > max_fr3)
            max_fr3 = dma_ir[3];
    }

    void IRsensor::SetIRSensorData()
    {
        ir_value.fl1 = max_fl1;
        ir_value.fl2 = max_fl2;
        ir_value.fl3 = max_fl3;
        ir_value.fr1 = max_fr1;
        ir_value.fr2 = max_fr2;
        ir_value.fr3 = max_fr3;

        max_fl1 = 0;
        max_fl2 = 0;
        max_fl3 = 0;
        max_fr1 = 0;
        max_fr2 = 0;
        max_fr3 = 0;
    }

    bool IRsensor::StartInitialize()
    {
        if (dma_ir[1] > ir_start_base && dma_ir[4] > ir_start_base)
            return true;
        else
            return false;
    }

    void IRsensor::PrintLog()
    {
        printf("%lu, %lu, %lu, %lu, %lu, %lu\n",
               ir_value.fl1, ir_value.fl2, ir_value.fl3,
               ir_value.fr1, ir_value.fr2, ir_value.fr3);
    }
}