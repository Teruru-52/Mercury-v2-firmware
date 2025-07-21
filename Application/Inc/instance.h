/**
 * @file instance.h
 * @author Teruru-52
 */

#ifndef INSTANCE_H_
#define INSTANCE_H_

#include "Agent.h"
#include "Maze.h"
#include "controller/controller.h"
#include "hardware/encoder.h"
#include "hardware/imu.h"
#include "hardware/ir_sensor.h"
#include "hardware/motor.h"
#include "mouse_utils.h"

extern Maze maze;
extern Maze maze_backup;
extern Agent agent;
extern State state;
extern undercarriage::Controller controller;
extern hardware::IR_Value ir_value;
extern hardware::IRsensor ir_sensors;

#endif  // INSTANCE_H_