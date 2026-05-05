/**
 * @file DJI_Motor_task.h
 * @brief DJI M3508 task bridge APIs.
 */

#ifndef DJI_MOTOR_TASK_H
#define DJI_MOTOR_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

void DJI_M3508_Task_Init(void);

void DJI_M3508_Task(void *argument);

void DJI_M3508_CAN_RxCpltCallback(uint8_t *Rx_Data);

#ifdef __cplusplus
}
#endif

#endif
