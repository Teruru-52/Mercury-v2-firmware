/**
 * @file imu.h
 * @author Teruru-52
 */

#ifndef HARDWARE_IMU_H_
#define HARDWARE_IMU_H_

#include "main.h"

namespace hardware
{
    class IMU
    {
    public:
        IMU(float sampling_period);

        uint8_t read_byte(uint8_t reg);
        void write_byte(uint8_t reg, uint8_t data);

        void Initialize();
        void CalcOffset();
        void Update();
        void UpdateGyro();
        void UpdateAcc();
        float GetAngle();
        float GetAngularVelocity();
        float GetAccX();
        void ResetTheta();

    private:
        float sampling_period;
        // default
        const float gyro_factor = 16.4f;
        // const float gyro_factor = 16.6f;
        const float acc_factor = 8192.0f;
        const float g = 9.81f * 1e+3f; // mm/s^2

        float theta;
        float gyro_z;
        float pre_gyro_z;
        float acc_x;
        float offset_gz;
        float offset_ax;
    };
}
#endif // HARDWARE_IMU_H_