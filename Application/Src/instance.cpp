/**
 * @file instance.cpp
 * @author Teruru-52
 */

#include "instance.h"

const float sampling_period = 0.001f;
hardware::IMU imu(sampling_period);
hardware::Encoder encoder(sampling_period);
hardware::Motor motor;

const float ir_start_base = 2500;

const float ir_fl1_wall = 2220; // WALL_TIMING 0.8
const float ir_fl2_wall = 2160;
const float ir_fl3_wall = 2160;
const float ir_fr1_wall = 2150;
const float ir_fr2_wall = 2150;
const float ir_fr3_wall = 2150;
const float ir_fl1_base = 2220;
const float ir_fl2_base = 2220;
const float ir_fl3_base = 2220;
const float ir_fr1_base = 3650;
const float ir_fr2_base = 3550;
const float ir_fr3_base = 3550;
const float ir_slalom = 2420; // front wall correction (slalom)

hardware::IR_Value ir_value;
// for wall judgement
hardware::IR_Base ir_is_wall = {ir_fl1_wall, ir_fl2_wall, ir_fl3_wall, ir_fr1_wall, ir_fr2_wall, ir_fr3_wall};
// for front/side wall correction
hardware::IR_Base ir_ctrl_base = {ir_fl1_base, ir_fl2_base, ir_fl3_base, ir_fr1_base, ir_fr2_base, ir_fr3_base, ir_slalom};
hardware::IR_LogCoeff ir_log = {.a = 41.01f, .b = 1.314e-5f, .c = -0.02817f, .d = 262.2f};

hardware::IR_Param ir_param = {ir_is_wall, ir_ctrl_base, ir_log};
hardware::IRsensor ir_sensors(ir_start_base, &ir_is_wall);