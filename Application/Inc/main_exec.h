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

    void init();
    void update();
    void updateIR();

    void printLog();

#ifdef __cplusplus
};
#endif

#endif // MAIN_EXEC_H_