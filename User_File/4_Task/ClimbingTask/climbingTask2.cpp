#include "climbingTask2.h"
#include "cmsis_os2.h"
#include "main.h"
#include "Laser_Task.h"
#include "drv_uart.h"  

// 实例化全局唯一的 C++ 控制器对象
ClimbingController climbingCtrl;

ClimbingController &Climbing_Get_Controller(void)
{
    return climbingCtrl;
}

void Climbing_CAN_Rx_Dispatch(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    climbingCtrl.CAN_RxCallback(Header.Identifier, Buffer);
}

uint8_t Climbing_Is_Finding_Edge_Front(void) { return climbingCtrl.IsFindingEdgeFront(); }
uint8_t Climbing_Is_Finding_Edge_Rear(void) { return climbingCtrl.IsFindingEdgeRear(); }

// -----------------------------------------
// FreeRTOS 任务主循环
// -----------------------------------------
void ClimbingTask(void)
{
    // 任务入口里完成一次初始化
    climbingCtrl.Init(&hfdcan1);

    // 初始化激光串口 
    LaserTask_Init();
    
    //既然平时不需要测距，初始化完立刻强行掐断硬件接收
    HAL_UART_AbortReceive(&huart10);
    HAL_UART_AbortReceive(&huart1);

    osDelay(2000); // 等待系统硬件与总线稳定
    Climbing_Init_Pose_Start(); // 默认进入初始站立姿态
    
    uint8_t last_laser_front = 0;  
    uint8_t last_laser_rear = 0;    //记录后激光状态

    for (;;)
    {
        // 1ms 周期: 先推进自动状态, 再执行主控制计算
        climbingCtrl.AutoTask1ms();
        climbingCtrl.TaskEntry1ms();

        // ------------------ 管控前腿激光 (huart1) ------------------
        uint8_t current_laser_front = Climbing_Is_Finding_Edge_Front();
        if (current_laser_front != last_laser_front) {
            if (current_laser_front == 1) UART_Reinit(&huart1); 
            else HAL_UART_AbortReceive(&huart1); 
            last_laser_front = current_laser_front; 
        }

        // ------------------ 管控后腿激光 (huart10) ------------------
        uint8_t current_laser_rear = Climbing_Is_Finding_Edge_Rear();
        if (current_laser_rear != last_laser_rear) {
            if (current_laser_rear == 1) UART_Reinit(&huart10); 
            else HAL_UART_AbortReceive(&huart10); 
            last_laser_rear = current_laser_rear; 
        }
        osDelay(1);
    }
}

// -----------------------------------------
// 历史遗留兼容接口
// -----------------------------------------
void Climbing_Task_Init(void)
{
    climbingCtrl.Init(&hfdcan1);
}

void Climbing_Task_Entry(void)
{
    climbingCtrl.TaskEntry1ms();
}

// -----------------------------------------
// 自动流程封装 (供外部 C 代码或上位机调用)
// -----------------------------------------
void Climbing_Auto_Start(void)                  { climbingCtrl.AutoStart40cm(); }
void Climbing_Auto_Start_20cm(void)             { climbingCtrl.AutoStart20cm(); }
void Climbing_Auto_Start_40cm(void)             { climbingCtrl.AutoStart40cm(); }
void Climbing_Prepare_40cm(void)                { climbingCtrl.Prepare40cm(); }
void Climbing_Auto_Start_From_Touch_20cm(void)  { climbingCtrl.AutoStartFromTouch20cm(); }
void Climbing_Auto_Start_From_Touch_40cm(void)  { climbingCtrl.AutoStartFromTouch40cm(); }

void Climbing_Descend_Auto_Start(void)          { climbingCtrl.DescendAutoStart20cm(); }
void Climbing_Descend_Auto_Start_20cm(void)     { climbingCtrl.DescendAutoStart20cm(); }
void Climbing_Descend_Auto_Start_40cm(void)     { climbingCtrl.DescendAutoStart40cm(); }

void Climbing_Init_Pose_Start(void)             { climbingCtrl.InitPoseStart(); }
void Climbing_WeaponRodDockStart(void)          { climbingCtrl.WeaponRodDockStart(); }
void Climbing_WeaponHeadClampStart(void)        { climbingCtrl.WeaponHeadClampStart(); }

// -----------------------------------------
// 系统状态与控制封装
// -----------------------------------------
void Climbing_Auto_Task_1ms(void)
{
    climbingCtrl.AutoTask1ms();
}

uint8_t Climbing_Is_Auto_Running(void)
{
    return climbingCtrl.IsAutoRunning() ? 1 : 0;
}

void Climbing_Descend_Manual_Next(void)
{
    climbingCtrl.DescendManualNext();
}

void Climbing_Emergency_Stop(void)
{
    climbingCtrl.EmergencyStop();
}

ClimbingState_e Climbing_Get_State(void)
{
    return climbingCtrl.GetState();
}
