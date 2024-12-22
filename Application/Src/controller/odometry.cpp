/**
 * @file odometry.cpp
 * @author Teruru-52
 */

#include "controller/odometry.h"

namespace undercarriage
{
  Odometory::Odometory(float sampling_period)
      : encoder(sampling_period),
        imu(sampling_period),
        sampling_period(sampling_period) {}

  void Odometory::Initialize()
  {
    imu.Initialize();
    imu.CalcOffset();
    ResetEncoder();
    Reset();
  }

  void Odometory::Reset()
  {
    cur_pos.x = 0;
    cur_pos.y = 0;
    length = 0;
    pre_length = 0;
    // ResetTheta();
  }

  void Odometory::ResetTheta()
  {
    cur_pos.th = 0.0;
    imu.ResetTheta();
  }

  void Odometory::Update()
  {
    imu.Update();
    encoder.Update();

    vel_x = encoder.GetVelocity();
    acc_x = imu.GetAccX();
    cur_pos.th = imu.GetAngle();
    cur_vel.th = imu.GetAngularVelocity();

    cur_vel.x = vel_x;
    cur_vel.y = 0.0;

    length = encoder.GetPosition();
    cur_pos.x += (pre_length + length) * arm_cos_f32(cur_pos.th) * 0.5f;
    cur_pos.y += (pre_length + length) * arm_sin_f32(cur_pos.th) * 0.5f;
    pre_length = length;
  }

  void Odometory::OutputLog()
  {
    printf("%.3f, %.3f, %.3f, %.3f\n", cur_pos.x, cur_pos.y, cur_pos.th, acc_x);
  }
} //  namespace undercarriage