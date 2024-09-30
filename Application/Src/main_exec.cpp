/*
 * main_exec.cpp
 *
 *      Author: Teruru-52
 */

#include "main_exec.h"
#include "instance.h"

void init()
{
    // imu.Initialize();
    ir_sensors.startDMA();
}

void update()
{
    // imu.Update();
    // encoder.update();
    ir_sensors.update();
    // motor.Drive(0.5f, 0.5f);
}

void updateIR()
{
    ir_sensors.UpdateLeftValue();
    ir_sensors.UpdateRightValue();
}

void printLog()
{
    ir_sensors.printLog();
}