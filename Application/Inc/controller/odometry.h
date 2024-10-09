/**
 * @file odometry.h
 * @author Teruru-52
 */

#ifndef ODOMETORY_H_
#define ODOMETORY_H_

#include "hardware/encoder.h"
#include "hardware/imu.h"
#include "pose.h"

namespace undercarriage
{
    class Odometory
    {
    private:
        hardware::Encoder encoder;
        hardware::IMU imu;

        float sampling_period;       // [s]
        ctrl::Pose cur_pos{0, 0, 0}; // absolute coordinates
        ctrl::Pose cur_vel{0, 0, 0}; // robot coordinates
        float vel_x;
        float pre_length{0.0};
        float acc_x;
        float length{0.0};

    public:
        Odometory(float sampling_period);

        void Initialize();
        void Update();
        // void UpdateIMU() { imu.Update(); };
        void Reset();
        void ResetTheta();
        void ResetEncoder() { encoder.Reset(); };
        void OverWritePos(const ctrl::Pose cur_p) { cur_pos = cur_p; };
        int16_t GetPulseL() { return encoder.GetPulseL(); };
        int16_t GetPulseR() { return encoder.GetPulseR(); };
        void OutputLog();

        ctrl::Pose GetPosition() { return cur_pos; };
        ctrl::Pose GetVelocity() { return cur_vel; };
        float GetAccX() { return acc_x; };
        float GetLength() { return length; };
    };
} //  namespace undercarriage
#endif //  ODOMETORY_H_