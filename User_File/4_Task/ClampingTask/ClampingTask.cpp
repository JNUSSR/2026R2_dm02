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

    osDelay(5000); // 等待系统稳定
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
 */
void Clamping_Auto_Start(void)
{
    // 停止底盘
    Chassis_Set_Target(0.0f,0.0f,0.0f);
    osDelay(500);
    // 确保夹爪移动到水平位置
    Clamping_Auto_Adjust();
    // 前进一段固定距离（0.3m/s*1s=30cm）
    Chassis_Set_Target(0.3f,0.0f,0.0f);
    osDelay(980);
    Chassis_Set_Target(0.0f,0.0f,0.0f);
    osDelay(500);
    // 稍微放低一点
    Clamping_Get_Controller().MoveToAngle(CLAMPING_TARGET_ANGLE_CLAMP_RAD * 0.95);
    osDelay(1000);
    // 夹取
    clampingCtrl.OpenSolenoid();
    osDelay(100);
    // 分两步移动到对接位置
    Clamping_Get_Controller().MoveToAngle(CLAMPING_TARGET_ANGLE_DOCK_RAD * 0.75f);
    osDelay(1000);
    Clamping_Get_Controller().MoveToDockAngle();
    osDelay(500);
    // 后退一段固定距离（-0.3m/s*1s=-30cm）
    Chassis_Set_Target(-0.3f,0.0f,0.0f);
    osDelay(980);
    Chassis_Set_Target(0.0f,0.0f,0.0f);    
    // 转180度
    Chassis_Set_Target(0.0f,0.0f,PI/2);
    osDelay(2248);
    Chassis_Set_Target(0.0f,0.0f,0.0f);
    // 放低到对接位置
    Climbing_WeaponRodDockStart();
}

void Clamping_Auto_Adjust(void)
{
    Climbing_WeaponHeadClampStart();
    clampingCtrl.MoveToClampAngle();
    clampingCtrl.ReleaseSolenoid();
}
