/**
 * @file encoder.h
 * @author Teruru-52
 */

#ifndef HARDWARE_ENCODER_H_
#define HARDWARE_ENCODER_H_

#include "main.h"

namespace hardware
{
    class Encoder
    {
    public:
        Encoder(float sampling_period);

        void Update();
        void Reset();
        void Update_L();
        void Update_R();
        int16_t GetPulseL();
        int16_t GetPulseR();
        float GetAngularVelocity(int16_t pulse);
        float GetAngle(int16_t pulse);
        float GetVelocity();
        float GetPosition();

    private:
        const float gear_ratio = 11.0f / 43.0f;
        float sampling_period; // [s]
        // const float tire_radius = 12.7f; // [mm]
        const float tire_radius = 12.8f; // [mm]
        // const float tire_radius = 12.9f; // [mm]
        // const float tire_radius = 13.0f; // [mm]
        // const float tire_radius = 13.1f; // [mm]
        // const float tire_radius_l = tire_radius * 1.1f; // [mm]
        const float tire_radius_l = tire_radius; // [mm]
        const float tire_radius_r = tire_radius; // [mm]
        const float ppr = 1024.0f;
        float coeff_pulse2angle;
        float coeff_pulse2vel;
        int16_t pulse_left;
        int16_t pulse_right;
    };
} // namespace hardware

#endif // HARDWARE_ENCODER_HPP_