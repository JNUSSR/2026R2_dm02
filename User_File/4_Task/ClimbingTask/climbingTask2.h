#ifndef TEST_FEEDBACK_CLIMBINGTASK2_H
#define TEST_FEEDBACK_CLIMBINGTASK2_H

#include "drv_can.h"

#ifdef __cplusplus
#include "climbing_controller.h"
extern "C" {
#endif

// -----------------------------------------
// OS 任务与 CAN 接收接口
// -----------------------------------------
void ClimbingTask(void);
void Climbing_CAN_Rx_Dispatch(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer);

void Climbing_Task_Init(void);
void Climbing_Task_Entry(void);

// -----------------------------------------
// 自动流程对外调用接口
// -----------------------------------------
void Climbing_Auto_Start(void);
void Climbing_Auto_Start_20cm(void);
void Climbing_Auto_Start_40cm(void);
void Climbing_Prepare_40cm(void);
void Climbing_Auto_Start_From_Touch_20cm(void);
void Climbing_Auto_Start_From_Touch_40cm(void);
void Climbing_Descend_Auto_Start(void);
void Climbing_Descend_Auto_Start_20cm(void);
void Climbing_Descend_Auto_Start_40cm(void);
void Climbing_Init_Pose_Start(void);

void Climbing_WeaponRodDockStart(void);
void Climbing_WeaponHeadClampStart(void);

// -----------------------------------------
// 系统状态与控制接口
// -----------------------------------------
void Climbing_Auto_Task_1ms(void);
uint8_t Climbing_Is_Auto_Running(void);
void Climbing_Descend_Manual_Next(void);
void Climbing_Emergency_Stop(void);

#ifdef __cplusplus
ClimbingState_e Climbing_Get_State(void);
} // extern "C"

// 允许 C++ 文件获取控制器实例引用
ClimbingController &Climbing_Get_Controller(void);
#endif

#endif // TEST_FEEDBACK_CLIMBINGTASK2_H