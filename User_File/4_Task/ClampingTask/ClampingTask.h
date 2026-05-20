#ifndef R2_USER_TASK_CLAMPINGTASK_CLAMPINGTASK_H
#define R2_USER_TASK_CLAMPINGTASK_CLAMPINGTASK_H

#include "drv_can.h"

#ifdef __cplusplus
#include "Clamping_controller.h"
extern "C" {
#endif

void ClampingTask(void);
void Clamping_Task_Init(void);
void Clamping_CAN_Rx_Dispatch(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer);

void Clamping_Auto_Start(void);
void Clamping_Auto_Adjust(void);

#ifdef __cplusplus
}

ClampingController &Clamping_Get_Controller(void);

#endif

#endif
