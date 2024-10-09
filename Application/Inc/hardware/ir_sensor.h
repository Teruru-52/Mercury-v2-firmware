/**
 * @file ir_sensor.h
 * @author Teruru-52
 */

#ifndef HARDWARE_IR_SENSOR_H_
#define HARDWARE_IR_SENSOR_H_

#include "main.h"
#include "hardware/led.h"

namespace hardware
{
    struct IR_Value
    {
        uint32_t fl1;
        uint32_t fl2;
        uint32_t fl3;
        uint32_t fr1;
        uint32_t fr2;
        uint32_t fr3;
    };

    struct IR_Base
    {
        float fl1;
        float fl2;
        float fl3;
        float fr1;
        float fr2;
        float fr3;
        float slalom;
    };

    struct IR_LogCoeff
    {
        float a;
        float b;
        float c;
        float d;
    };

    struct IR_Param
    {
        IR_Base is_wall;
        IR_Base ctrl_base;
        IR_LogCoeff log;
    };

    class IRsensor
    {
    private:
        const int sampling_count = 84;
        float ir_start_base;

        uint16_t dma_ir[6];

        uint32_t max_fl1 = 0;
        uint32_t max_fl2 = 0;
        uint32_t max_fl3 = 0;
        uint32_t max_fr1 = 0;
        uint32_t max_fr2 = 0;
        uint32_t max_fr3 = 0;

        IR_Value ir_value;
        IR_Base *ir_is_wall;

    public:
        IRsensor(float ir_start_base, IR_Base *ir_is_wall);

        void OnLed();
        void OffLed();

        void UI_led_onoff(const IR_Value &ir_value);
        void UI_led_off();
        void PrintWalldata(const IR_Value &ir_value);

        void StartDMA();
        void UpdateLeftValue();
        void UpdateRightValue();
        void SetIRSensorData();
        IR_Value GetIRSensorData() { return ir_value; };
        bool StartInitialize();
        void PrintLog();
    };
}
#endif //  HARDWARE_IR_SENSOR_H_