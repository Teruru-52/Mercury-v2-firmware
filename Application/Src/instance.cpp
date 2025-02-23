/**
 * @file instance.cpp
 * @author Teruru-52
 */

#include "instance.h"

Maze maze;
Maze maze_backup;
Agent agent(maze);
State state;

const float ir_start_base = 3300;

// home
// const float ir_fl1_wall = 2150; // WALL_TIMING 0.8
// const float ir_fl2_wall = 2500;
// const float ir_fl3_wall = 2300;
// const float ir_fr1_wall = 2200;
// const float ir_fr2_wall = 2800;
// const float ir_fr3_wall = 2300;
// const float ir_fl1_base = 2220;
// const float ir_fl2_base = 2800;
// const float ir_fl3_base = 3150;
// const float ir_fr1_base = 3650;
// const float ir_fr2_base = 3050;
// const float ir_fr3_base = 3000;
// const float ir_slalom = 2450; // front wall correction (slalom)

// science tokyo
// const float ir_fl1_wall = 2150; // WALL_TIMING 0.8
// const float ir_fl2_wall = 2500;
// const float ir_fl3_wall = 2300;
// const float ir_fr1_wall = 2200;
// const float ir_fr2_wall = 2800;
// const float ir_fr3_wall = 2300;
// const float ir_fl1_base = 2220;
// const float ir_fl2_base = 2800;
// const float ir_fl3_base = 3400;
// const float ir_fr1_base = 3650;
// const float ir_fr2_base = 3050;
// const float ir_fr3_base = 3180;
// const float ir_slalom = 2420; // front wall correction (slalom)

// Contest
const float ir_fl1_wall = 2150; // WALL_TIMING 0.8
const float ir_fl2_wall = 2500;
const float ir_fl3_wall = 2300;
const float ir_fr1_wall = 2200;
const float ir_fr2_wall = 2800;
const float ir_fr3_wall = 2300;
const float ir_fl1_base = 2220;
const float ir_fl2_base = 2800;
const float ir_fl3_base = 3150;
const float ir_fr1_base = 3650;
const float ir_fr2_base = 3050;
const float ir_fr3_base = 3000;
const float ir_slalom = 2400; // front wall correction (slalom)

hardware::IR_Value ir_value;
// for wall judgement
hardware::IR_Base ir_is_wall = {ir_fl1_wall, ir_fl2_wall, ir_fl3_wall, ir_fr1_wall, ir_fr2_wall, ir_fr3_wall};
// for front/side wall correction
hardware::IR_Base ir_ctrl_base = {ir_fl1_base, ir_fl2_base, ir_fl3_base, ir_fr1_base, ir_fr2_base, ir_fr3_base, ir_slalom};
// hardware::IR_LogCoeff ir_log = {.a = 41.01f, .b = 1.314e-5f, .c = -0.02817f, .d = 262.2f};
hardware::IR_LogCoeff ir_log = {.a = 41.0f, .b = 1.3e-05f, .c = -0.028f, .d = 262.1f};

hardware::IR_Param ir_param = {ir_is_wall, ir_ctrl_base, ir_log};
hardware::IRsensor ir_sensors(ir_start_base, &ir_is_wall);

const float sampling_period = 0.001f;
const float control_period = 0.001f;

undercarriage::Odometory odom(sampling_period);

PID pid_angle(4.0f, 0.0, 0.0, 0.0, control_period);
// PID pid_rotational_vel(1.1976f, 85.1838f, -0.00099f, 0.0039227f, control_period);
// PID pid_rotational_vel(1.3f, 30.0f, 0.001f, 0.0004f, control_period);
PID pid_rotational_vel(0.3f, 30.0f, 0.0f, 0.0004f, control_period);
// PID pid_traslational_vel(0.0068176f, 0.0820249f, -0.000033349f, 0.023191f, control_period);
PID pid_traslational_vel(0.03f, 0.01f, 0.0f, 0.02f, control_period);
PID pid_ir_front_left(0.0008f, 0.000005f, 0.0, 0.0, control_period);
PID pid_ir_front_right(0.0008f, 0.000005f, 0.0, 0.0, control_period);
// PID pid_ir_side(0.002f, 0.000, 0.0, 0.0, control_period);
PID pid_ir_side(0.001f, 0.0, 0.0, 0.0, control_period);

PID_Instances pid(&pid_angle,
                  &pid_rotational_vel,
                  &pid_traslational_vel,
                  &pid_ir_front_left,
                  &pid_ir_front_right,
                  &pid_ir_side);

undercarriage::Kanayama kanayama(3.0f, 3.0e-6f, 1.5e-3f);
undercarriage::DynamicFeedback dynamic_feedback(10.0f, 0.5f, control_period);
undercarriage::TimeVaryingFeedback time_varying_feedback(1.0f, 1e-3f);
undercarriage::TrackerBase *tracker = &kanayama;
// undercarriage::TrackerBase *tracker = &dynamic_feedback;
// undercarriage::TrackerBase *tracker = &time_varying_feedback;

// translational velocity
trajectory::Velocity velocity = {.v1 = 200.0f, .v2 = 300.0f, .v3 = 350.0f, .v4 = 400.0f, .v5 = 1000.0f};

trajectory::Parameter acc_param1 = {.v_max = 2.0e+2f, .a_max = 1.5e+4f, .j_max = 2.0e+5f};
trajectory::Parameter acc_param2 = {.v_max = 3.0e+2f, .a_max = 1.5e+4f, .j_max = 2.0e+5f};
trajectory::Parameter acc_param3 = {.v_max = 3.5e+2f, .a_max = 1.5e+4f, .j_max = 2.0e+5f};
trajectory::Parameter acc_param4 = {.v_max = 4.0e+3f, .a_max = 1.5e+4f, .j_max = 2.5e+5f};
trajectory::Parameter acc_param5 = {.v_max = 1.5e+3f, .a_max = 2.0e+4f, .j_max = 2.5e+5f};
trajectory::Parameters acc_params = {.run1 = acc_param1, .run2 = acc_param2, .run3 = acc_param3, .run4 = acc_param4, .run5 = acc_param5};

trajectory::Parameter slalom_param1 = {.v_max = 2.0f * M_PI, .a_max = 30.0f * M_PI, .j_max = 1.0e+3f * M_PI};
trajectory::Parameter slalom_param2 = {.v_max = 5.0f * M_PI, .a_max = 40.0f * M_PI, .j_max = 1.5e+3f * M_PI};
trajectory::Parameter slalom_param3 = {.v_max = 8.0f * M_PI, .a_max = 50.0f * M_PI, .j_max = 1.5e+3f * M_PI};
trajectory::Parameter slalom_param4 = {.v_max = 12.0f * M_PI, .a_max = 60.0f * M_PI, .j_max = 2.0e+3f * M_PI};
trajectory::Parameter slalom_param5 = {.v_max = 15.0f * M_PI, .a_max = 70.0f * M_PI, .j_max = 2.5e+3f * M_PI};
trajectory::Parameters slalom_params = {.run1 = slalom_param1, .run2 = slalom_param2, .run3 = slalom_param3, .run4 = slalom_param4, .run5 = slalom_param5};

trajectory::Slalom slalom(&velocity, &slalom_params);
trajectory::Acceleration acc(&velocity, &acc_params);

undercarriage::Controller controller(&odom,
                                     &pid,
                                     tracker,
                                     &slalom,
                                     &acc,
                                     &ir_param,
                                     &velocity);