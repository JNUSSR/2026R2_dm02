#include "ClampingTask.h"

#include "cmsis_os2.h"
#include "main.h"
#include "ChassisTask.h"

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

void Clamping_Auto_Start(void)
{
    Clamping_Auto_Adjust();
    osDelay(1000);
    Chassis_Set_Target(0.1f,0.0f,0.0f);
    osDelay(1400);
    Chassis_Set_Target(0.0f,0.0f,0.0f);
    osDelay(1000);
    clampingCtrl.OpenSolenoid();
    osDelay(1000);
    Chassis_Set_Target(-0.1f,0.0f,0.0f);
    osDelay(1400);
    Chassis_Set_Target(0.0f,0.0f,0.0f);
}

void Clamping_Auto_Adjust(void)
{
    clampingCtrl.MoveToClampAngle();
    clampingCtrl.ReleaseSolenoid();
}
