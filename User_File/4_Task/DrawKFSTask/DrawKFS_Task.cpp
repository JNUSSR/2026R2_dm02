//
// Created by chengfeng on 2026/5/2.
//

#include "DrawKFS_Task.h"
#include "arm.h"
#include "cmsis_os2.h"
#include "drv_can.h"
#include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"
#include "main.h"
#include "stm32h723xx.h"
#include "stm32h7xx_hal_gpio.h"
#include "uart_printf.h"
#include <math.h>

// ================= 左臂电机 =================
Class_Motor_DJI_C620 Motor_Z;
Class_Motor_DJI_C610 Motor_X;
Class_Motor_DJI_C620 Motor_R;

// ================= 右臂电机 =================
Class_Motor_DJI_C620 Motor_Z_R;
Class_Motor_DJI_C610 Motor_X_R;
Class_Motor_DJI_C620 Motor_R_R;

#define ARRAY_LEN(arr) (sizeof(arr) / sizeof(arr[0]))

const ArmStep IDLE[] = {
    {0.1f, 5.0f, 0.0f, 0.0f,M_PI / 2, 5.0f, nullptr}
};

const ArmStep DrawKFS[] = {
    //  Z目标, Z耗时 |  X目标, X耗时 |  R目标, R耗时
    {
        0.20f, 1.0f, 0.0f, 0.0f, 0.0, 0.0f,
        [](uint8_t arm_id) { 
            if (arm_id == 0) {
                HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13, GPIO_PIN_SET);
            }
            else if (arm_id == 1) {
                HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2, GPIO_PIN_SET);
            }
        }
    }, // 步骤1
    {0.20f, 0.0f, 0.55f, 2.5f, M_PI / 2, 1.0f, nullptr}, // 步骤2
    {0.20f, 0.0f, 0.55f, 1.0f, M_PI / 2, 0.0f, nullptr}, // 步骤3，等待1s吸取
    {0.20f, 0.0f, 0.0f, 4.0f, M_PI / 2, 0.0f, nullptr}, // 步骤4
    {0.50f, 1.5f, 0.0f, 0.0f, M_PI / 2, 0.0f, nullptr}, // 步骤5
    {
    0.50f, 0.0f, 0.0f, 0.0f, M_PI, 1.0f,
        [](uint8_t arm_id) { 
            if (arm_id == 0) {
                HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13, GPIO_PIN_SET);
            }
            else if (arm_id == 1) {
                HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2, GPIO_PIN_SET);
            }
        }
    }
};

const ArmStep DrawKFS_40cm[] = {
    //  Z目标, Z耗时 |  X目标, X耗时 |  R目标, R耗时
    {
        0.40f, 2.5f, 0.0f, 0.0f, 0, 0.0f,
        [](uint8_t arm_id) { 
            if (arm_id == 0) {
                HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13, GPIO_PIN_SET);
            }
            else if (arm_id == 1) {
                HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2, GPIO_PIN_SET);
            }
        }
    }, // 步骤1
    {0.40f, 0.0f, 0.55f, 2.5f, M_PI / 2, 2.0f, nullptr}, // 步骤2
    {0.40f, 0.0f, 0.55f, 1.0f, M_PI / 2, 0.0f, nullptr}, // 步骤3，等待1s吸取
    {0.40f, 0.0f, 0.0f, 4.0f, M_PI / 2, 0.0f, nullptr}, // 步骤4
    {0.50f, 2.0f, 0.0f, 0.0f, M_PI / 2, 0.0f, nullptr}, // 步骤5
    {
        0.50f, 0.0f, 0.0f, 0.0f, M_PI, 1.0f,
        [](uint8_t arm_id) { 
            if (arm_id == 0) {
                HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13, GPIO_PIN_SET);
            }
            else if (arm_id == 1) {
                HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2, GPIO_PIN_SET);
            }
        }
    }
};

const ArmStep DrawKFS_Below20cm[] = {
    //  Z目标, Z耗时 |  X目标, X耗时 |  R目标, R耗时
    {
        0.0f, 2.0f, 0.0f, 0.0f, M_PI / 2, 3.0f,
        [](uint8_t arm_id) { 
            if (arm_id == 0) {
                HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13, GPIO_PIN_SET);
            }
            else if (arm_id == 1) {
                HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2, GPIO_PIN_SET);
            }
        }
    }, // 步骤1
    {0.0f, 0.0f, 0.55f, 2.5f, M_PI / 2, 0.0f, nullptr}, // 步骤2
    {0.50f, 2.0f, 0.55f, 1.0f, M_PI / 2, 0.0f, nullptr}, // 步骤3，等待1s吸取
    {0.50f, 2.0f, 0.0f, 2.5f, M_PI / 2, 0.0f, nullptr}, // 步骤4
    {
        0.50f, 0.0f, 0.0f, 0.0f, M_PI, 1.0f,
        [](uint8_t arm_id) { 
            if (arm_id == 0) {
                HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13, GPIO_PIN_SET);
            }
            else if (arm_id == 1) {
                HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2, GPIO_PIN_SET);
            }
        }
    }
};

const ArmStep PutKFS_OrderTwo[] = {
    {0.70f,2.0f, 0.0f, 0.0f, M_PI / 2 * 1.05, 2.0f,
    [](uint8_t arm_id) { 
        if (arm_id == 0) {
            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13, GPIO_PIN_SET);
        }
        else if (arm_id == 1) {
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2, GPIO_PIN_SET);
        }
    }},// 步骤1
    {1.0f,2.0f,0.0f,0.0f, M_PI / 2 * 1.05, 0.0f,nullptr},//步骤2
    {1.0f,0.0f,0.50f,2.0f, M_PI / 2 * 1.05, 0.0f,nullptr},//步骤3
    {1.0f,3.0f,0.50f,3.0f, M_PI / 2, 0.2f,
    [](uint8_t arm_id) { 
        if (arm_id == 0) {
            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13, GPIO_PIN_RESET);
        }
        else if (arm_id == 1) {
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2, GPIO_PIN_RESET);
        }
    }},//步骤4，等待3s
    {0.0f, 5.0f, 0.0f, 2.5f, 0.0f, 3.0f, nullptr}//步骤5，回到初始位置
};

// ================= 左臂 Flags =================
bool drawkfs_flag = false;
bool drawkfs_40cm_flag = false;
bool drawkfs_below20cm_flag = false;
bool idle_flag = false;
bool putkfs_ordertwo_flag = false;

// ================= 右臂 Flags =================
bool right_drawkfs_flag = false;
bool right_drawkfs_40cm_flag = false;
bool right_drawkfs_below20cm_flag = false;
bool right_idle_flag = false;
bool right_putkfs_ordertwo_flag = false;

void DrawKFS_Task() {
    // ----------------- 左臂初始化 -----------------
    Motor_Z.PID_Omega.Init(1500.0f, 600.0f, 0.0f, 0.0f, 3000.0f, 8000.0f);
    Motor_Z.PID_Angle.Init(20.0f, 0.0f, 0.0f, 0.0f, 10.0f, 15.0f);
    Motor_Z.Init(&hfdcan1, Motor_DJI_ID_0x205, Motor_DJI_Control_Method_ANGLE);

    // 同步 R2_Z: DrawKFS 的 X/R 在 CAN1 的 0x206/0x207
    Motor_X.PID_Omega.Init(1000.0f, 200.0f, 0.0f, 0.0f, 2000.0f, 4000.0f);
    Motor_X.PID_Angle.Init(10.0f, 0.0f, 0.0f, 0.0f, 10.0f, 10.0f);
    Motor_X.Init(&hfdcan1, Motor_DJI_ID_0x206, Motor_DJI_Control_Method_ANGLE);

    Motor_R.PID_Omega.Init(1500.0f, 600.0f, 0.0f, 0.0f, 3000.0f, 8000.0f);
    Motor_R.PID_Angle.Init(20.0f, 0.0f, 0.0f, 0.0f, 10.0f, 15.0f);
    Motor_R.Init(&hfdcan1, Motor_DJI_ID_0x207, Motor_DJI_Control_Method_ANGLE);

    // ----------------- 右臂初始化 -----------------
    Motor_Z_R.PID_Omega.Init(1500.0f, 600.0f, 0.0f, 0.0f, 3000.0f, 8000.0f);
    Motor_Z_R.PID_Angle.Init(20.0f, 0.0f, 0.0f, 0.0f, 10.0f, 15.0f);
    Motor_Z_R.Init(&hfdcan3, Motor_DJI_ID_0x201, Motor_DJI_Control_Method_ANGLE); 

    Motor_X_R.PID_Omega.Init(1000.0f, 200.0f, 0.0f, 0.0f, 2000.0f, 4000.0f);
    Motor_X_R.PID_Angle.Init(10.0f, 0.0f, 0.0f, 0.0f, 10.0f, 10.0f);
    Motor_X_R.Init(&hfdcan3, Motor_DJI_ID_0x202, Motor_DJI_Control_Method_ANGLE);

    Motor_R_R.PID_Omega.Init(1500.0f, 600.0f, 0.0f, 0.0f, 3000.0f, 8000.0f);
    Motor_R_R.PID_Angle.Init(20.0f, 0.0f, 0.0f, 0.0f, 10.0f, 15.0f);
    Motor_R_R.Init(&hfdcan3, Motor_DJI_ID_0x203, Motor_DJI_Control_Method_ANGLE);

    // ----------------- 适配器配置 -----------------
    MotorAdapter_C620 Adapter_Z(Motor_Z);
    MotorAdapter_C610 Adapter_X(Motor_X);
    MotorAdapter_C620 Adapter_R(Motor_R);

    MotorAdapter_C620 Adapter_Z_R(Motor_Z_R);
    MotorAdapter_C610 Adapter_X_R(Motor_X_R);
    MotorAdapter_C620 Adapter_R_R(Motor_R_R);

    float dt = 0.001;
    // 左臂关节
    PlannedJoint Joint_Z(Adapter_Z, 0, 1.1, 0, 1, 0.5 * 1 / 0.017f, dt); //双倍抬升
    PlannedJoint Joint_X(Adapter_X, -0.03, 0.6, 0, 1, 1 / 0.0165f, dt);
    PlannedJoint Joint_R(Adapter_R, -M_PI,2*M_PI, 0, -1, 1, dt);

    // 右臂关节
    PlannedJoint Joint_Z_R(Adapter_Z_R, 0, 1.1, 0, 1, 0.5 * 1 / 0.017f, dt); 
    PlannedJoint Joint_X_R(Adapter_X_R, -0.03, 0.6, 0, 1, 1 / 0.0165f, dt);
    PlannedJoint Joint_R_R(Adapter_R_R, -M_PI,2*M_PI, 0, -1, 1, dt);

    // 实例化动作播放器
    ArmSequencePlayer player_L(0, Joint_Z, Joint_X, Joint_R);
    ArmSequencePlayer player_R(1, Joint_Z_R, Joint_X_R, Joint_R_R);

    osDelay(2000);
    for (;;) {
        // ================== 左臂触发逻辑 ==================
        if (idle_flag && !player_L.IsPlaying()) {
            player_L.Play(IDLE, ARRAY_LEN(IDLE), true);
            idle_flag = false;
        }
        else if (drawkfs_40cm_flag && !player_L.IsPlaying()) {
            player_L.Play(DrawKFS_40cm, ARRAY_LEN(DrawKFS_40cm));
            drawkfs_40cm_flag = false;
        }
        else if (drawkfs_below20cm_flag && !player_L.IsPlaying()) {
            player_L.Play(DrawKFS_Below20cm, ARRAY_LEN(DrawKFS_Below20cm));
            drawkfs_below20cm_flag = false;
        }
        else if (drawkfs_flag && !player_L.IsPlaying()) {
            player_L.Play(DrawKFS, ARRAY_LEN(DrawKFS));
            drawkfs_flag = false;
        }
        else if (putkfs_ordertwo_flag && !player_L.IsPlaying()) {
            player_L.Play(PutKFS_OrderTwo, ARRAY_LEN(PutKFS_OrderTwo));
            putkfs_ordertwo_flag = false;
        }

        // ================== 右臂触发逻辑 ==================
        if (right_idle_flag && !player_R.IsPlaying()) {
            player_R.Play(IDLE, ARRAY_LEN(IDLE), true);
            right_idle_flag = false;
        }
        else if (right_drawkfs_40cm_flag && !player_R.IsPlaying()) {
            player_R.Play(DrawKFS_40cm, ARRAY_LEN(DrawKFS_40cm));
            right_drawkfs_40cm_flag = false;
        }
        else if (right_drawkfs_below20cm_flag && !player_R.IsPlaying()) {
            player_R.Play(DrawKFS_Below20cm, ARRAY_LEN(DrawKFS_Below20cm));
            right_drawkfs_below20cm_flag = false;
        }
        else if (right_drawkfs_flag && !player_R.IsPlaying()) {
            player_R.Play(DrawKFS, ARRAY_LEN(DrawKFS));
            right_drawkfs_flag = false;
        }
        else if (right_putkfs_ordertwo_flag && !player_R.IsPlaying()) {
            player_R.Play(PutKFS_OrderTwo, ARRAY_LEN(PutKFS_OrderTwo));
            right_putkfs_ordertwo_flag = false;
        }

        player_L.Update();
        player_R.Update();

        Joint_Z.Update();
        Joint_X.Update();
        Joint_R.Update();
        
        Joint_Z_R.Update();
        Joint_X_R.Update();
        Joint_R_R.Update();

        Motor_Z.TIM_Calculate_PeriodElapsedCallback();
        Motor_X.TIM_Calculate_PeriodElapsedCallback();
        Motor_R.TIM_Calculate_PeriodElapsedCallback();

        Motor_Z_R.TIM_Calculate_PeriodElapsedCallback();
        Motor_X_R.TIM_Calculate_PeriodElapsedCallback();
        Motor_R_R.TIM_Calculate_PeriodElapsedCallback();
        osDelay(1);
    }
}
