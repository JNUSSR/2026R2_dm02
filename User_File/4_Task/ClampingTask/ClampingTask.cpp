#include "ClampingTask.h"

#include "cmsis_os2.h"
#include "main.h"
#include "ChassisTask.h"
#include "ClimbingTask2.h"

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
    clampingCtrl.Init(&hfdcan1);

    osDelay(3000); // 等待系统稳定
    Clamping_Auto_Adjust(); // 夹爪举到水平（夹取）角度，夹爪释放
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

/**
 * @brief 一套夹取动作
 * @note 首先确保夹爪水平，然后前进，然后夹取，然后后退，然后停止
 */
void Clamping_Auto_Start(void)
{
    // 停止底盘
    Chassis_Set_Target(0.0f,0.0f,0.0f);
    osDelay(500);
    // 确保夹爪移动到水平位置
    Clamping_Auto_Adjust();
    osDelay(500);
    // 前进一段固定距离（0.2m/s*1.5s=30cm）
    Chassis_Set_Target(0.2f,0.0f,0.0f);
    osDelay(1500);
    Chassis_Set_Target(0.0f,0.0f,0.0f);
    osDelay(1000);
    // 执行夹取动作
    clampingCtrl.OpenSolenoid();
    osDelay(1000);
    // 移动到垂直（对接）位置
    Clamping_Get_Controller().MoveToDockAngle();
    osDelay(500);
    // 后退一段固定距离（-0.2m/s*1.5s=-30cm）
    Chassis_Set_Target(-0.2f,0.0f,0.0f);
    osDelay(1500);
    Chassis_Set_Target(0.0f,0.0f,0.0f);

    Climbing_Init_Pose_Start();
}

void Clamping_Auto_Adjust(void)
{
    clampingCtrl.MoveToClampAngle();
    clampingCtrl.ReleaseSolenoid();
}
