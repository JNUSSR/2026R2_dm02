#include "ClampingTask.h"

#include "cmsis_os2.h"
#include "main.h"

ClampingController clampingCtrl;

ClampingController &Clamping_Get_Controller(void)
{
    return clampingCtrl;
}

void Clamping_CAN_Rx_Dispatch(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    if (Header.Identifier == 0 || Buffer == nullptr)
    {
        return;
    }
    clampingCtrl.CAN_RxCallback(Header.Identifier, Buffer);
}

void ClampingTask(void)
{
    clampingCtrl.Init(&hfdcan2);
    for (;;)
    {
        clampingCtrl.TaskEntry1ms();
        osDelay(1);
    }
}

void Clamping_Task_Init(void)
{
    clampingCtrl.Init(&hfdcan1);
}
