/**
 * @file instance.h
 * @author Teruru-52
 */

#ifndef INSTANCE_H_
#define INSTANCE_H_

#include "Maze.h"
#include "Agent.h"
#include "mouse_utils.h"
#include "controller/controller.h"
#include "hardware/imu.h"
#include "hardware/encoder.h"
#include "hardware/motor.h"
#include "hardware/ir_sensor.h"

extern Maze maze;
extern Maze maze_backup;
extern Agent agent;
extern State state;
extern undercarriage::Controller controller;
extern hardware::IR_Value ir_value;
extern hardware::IRsensor ir_sensors;

#endif // INSTANCE_H_