#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "drv_can.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief 底盘任务初始化
 */
void Chassis_Task_Init(void);

/**
 * @brief 底盘任务入口（1ms周期）
 */
void Chassis_Task(void* argument);

/**
 * @brief CAN报文分发到轮式底盘模块
 */
void Chassis_CAN_Rx_Dispatch(FDCAN_HandleTypeDef* hfdcan, FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer);

/**
 * @brief 限制底盘速度
 */
void Chassis_Speed_Limit(float* vx, float* vy, float* vw);

/**
 * @brief 立即停止底盘运动
 */
void Chassis_Task_StopImmediate(void);

/**
 * @brief 立即更新底盘控制状态
 */
void Chassis_Task_Update(void);

/**
 * @brief 设置底盘目标速度（供MAVLink等上层调用）
 */
void Chassis_Set_Target(float vx, float vy, float vw);

#ifdef __cplusplus
}
#endif

#endif
