//
// Created by chengfeng on 2026/5/2.
//

#include "DrawKFS_Task.h"
#include "arm.h"
#include "cmsis_os2.h"
#include "drv_can.h"
#include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"
#include "main.h"
#include "uart_printf.h"

Class_Motor_DJI_C620 Motor_Z;
Class_Motor_DJI_C610 Motor_X;
Class_Motor_DJI_C610 Motor_R;

#define ARRAY_LEN(arr) (sizeof(arr) / sizeof(arr[0]))

const ArmStep IDLE[] = {
    {0.1f, 5.0f, 0.0f, 0.0f,M_PI / 2, 5.0f, nullptr}
};

const ArmStep DrawKFS[] = {
    //  Z目标, Z耗时 |  X目标, X耗时 |  R目标, R耗时
    {
        0.05f, 5.0f, 0.0f, 0.0f, M_PI / 2, 5.0f,
        []() { HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13, GPIO_PIN_SET); }
    }, // 步骤1
    {0.05f, 0.0f, 0.15f, 5.0f, M_PI / 2, 0.0f, nullptr}, // 步骤2
    {0.10f, 5.0f, 0.15f, 0.0f, M_PI / 2, 0.0f, nullptr}, // 步骤3
    {0.10f, 0.0f, 0.0f, 5.0f, M_PI / 2, 0.0f, nullptr}, // 步骤4
    {
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        []() { HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13, GPIO_PIN_RESET); }
    }
};

bool drawkfs_flag = false;
bool idle_flag = false;

// void CAN1_Motor_Call_Back(Struct_CAN_Rx_Buffer *Rx_Buffer) {
//     switch (Rx_Buffer->Header.StdId) {
//     }
// }

void DrawKFS_Task() {
    //BSP_Init(BSP_LED_1_ON | BSP_LED_8_ON);
    //CAN_Init(&hcan1, CAN1_Motor_Call_Back);
    //CAN_Init(&hcan2, CAN2_Motor_Call_Back);
    Motor_Z.PID_Omega.Init(1500.0f, 600.0f, 0.0f, 0.0f, 3000.0f, 8000.0f);
    Motor_Z.PID_Angle.Init(20.0f, 0.0f, 0.0f, 0.0f, 10.0f, 15.0f);
    Motor_Z.Init(&hfdcan2, Motor_DJI_ID_0x205, Motor_DJI_Control_Method_ANGLE);

    Motor_X.PID_Omega.Init(1000.0f, 200.0f, 0.0f, 0.0f, 1500.0f, 3000.0f);
    Motor_X.PID_Angle.Init(10.0f, 0.0f, 0.0f, 0.0f, 10.0f, 10.0f);
    Motor_X.Init(&hfdcan2, Motor_DJI_ID_0x206, Motor_DJI_Control_Method_ANGLE);

    Motor_R.PID_Omega.Init(1300.0f, 200.0f, 0.0f, 0.0f, 1500.0f, 3000.0f);
    Motor_R.PID_Angle.Init(15.0f, 0.0f, 0.0f, 0.0f, 10.0f, 15.0f);
    Motor_R.Init(&hfdcan2, Motor_DJI_ID_0x207, Motor_DJI_Control_Method_ANGLE);

    MotorAdapter_C620 Adapter_Z(Motor_Z);
    MotorAdapter_C610 Adapter_X(Motor_X);
    MotorAdapter_C610 Adapter_R(Motor_R);

    float dt = 0.001;
    PlannedJoint Joint_Z(Adapter_Z, 0, 0.4, 0, -1, 0.5 * 1 / 0.017f, dt); //双倍抬升
    PlannedJoint Joint_X(Adapter_X, 0, 0.32, 0, 1, 1 / 0.0165f, dt);
    PlannedJoint Joint_R(Adapter_R, 0,PI, 0, -1, 1, dt);

    // 实例化动作播放器
    ArmSequencePlayer player(Joint_Z, Joint_X, Joint_R);

    osDelay(2000);
    for (;;) {
        if (idle_flag && !player.IsPlaying()) {
            player.Play(IDLE,ARRAY_LEN(IDLE));
            idle_flag = false;
        }
        else if (drawkfs_flag && !player.IsPlaying()) {
            player.Play(DrawKFS, ARRAY_LEN(DrawKFS));
            drawkfs_flag = false;
        }

        player.Update();

        Joint_Z.Update();
        Joint_X.Update();
        Joint_R.Update();
        Motor_Z.TIM_Calculate_PeriodElapsedCallback();
        Motor_X.TIM_Calculate_PeriodElapsedCallback();
        Motor_R.TIM_Calculate_PeriodElapsedCallback();
        //TIM_CAN_PeriodElapsedCallback();
        osDelay(1);
    }
}
