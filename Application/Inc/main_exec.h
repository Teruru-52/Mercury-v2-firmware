/*
 * main_exec.h
 *
 *      Author: Teruru-52
 */

#ifndef MAIN_EXEC_H_
#define MAIN_EXEC_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include "main.h"

    void Startup();
    void Init();
    void UpdateUndercarriage();
    void SetIRSensor();
    void UpdateIRSensor();
    void UpdateMovement();
    void Notification();
    bool GetInterruptionFlag();
    void MazeSearch();
    void TimeAttack();
    void StateProcess();
    void FlashMaze();
    void LoadMaze();

    void PrintLog();

#ifdef __cplusplus
};
#endif

#endif // MAIN_EXEC_H_