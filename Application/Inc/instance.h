/**
 * @file instance.h
 * @author Teruru-52
 */

#ifndef INSTANCE_H_
#define INSTANCE_H_

#include "hardware/imu.h"
#include "hardware/encoder.h"
#include "hardware/motor.h"
#include "hardware/ir_sensor.h"

extern hardware::IMU imu;
extern hardware::Encoder encoder;
extern hardware::Motor motor;
extern hardware::IR_Value ir_value;
extern hardware::IRsensor ir_sensors;

#endif // INSTANCE_H_