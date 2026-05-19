/**
 * @file dvc_motor_dj_leso.cpp
 * @brief DJI电机LESO派生类
 */

#include "2_Device/Motor/Motor_DJI/Motor_DJI_LESO/dvc_motor_dji_leso.h"

/**
 * @brief 构造C610 LESO电机对象
 *
 * @param __LESO_Frequency LESO带宽
 * @param __LESO_Dt LESO采样周期
 * @param __LESO_B LESO输入增益
 */
Class_Motor_DJI_C610_LESO::Class_Motor_DJI_C610_LESO(float __LESO_Frequency, float __LESO_Dt, float __LESO_B):
    LESO_Frequency(__LESO_Frequency),
    LESO_Dt(__LESO_Dt),
    LESO_B(__LESO_B),
    Omega_Kp(0.0f),
    Omega_Current_Max(10000.0f),
    Position_Current_Max(10000.0f),
    Position_Kp(0.0f),
    Position_Kd(0.0f),
    Omega_ADRC(__LESO_Frequency, __LESO_Dt, __LESO_B, 0.0f, 10000.0f),
    Position_ADRC(__LESO_Frequency, __LESO_Dt, __LESO_B, 0.0f, 10000.0f)
{
}

/**
 * @brief 100ms存活检测回调, 电机失联时重置LESO
 *
 */
void Class_Motor_DJI_C610_LESO::TIM_100ms_Alive_PeriodElapsedCallback()
{
    Class_Motor_DJI_C610::TIM_100ms_Alive_PeriodElapsedCallback();

    if (Motor_DJI_Status == Motor_DJI_Status_DISABLE)
    {
        Reset_LESO();
    }
}

/**
 * @brief C610计算回调, 输出16bit电流刻度并叠加LESO补偿
 *
 */
void Class_Motor_DJI_C610_LESO::TIM_Calculate_PeriodElapsedCallback()
{
    switch (Motor_DJI_Control_Method)
    {
    case (Motor_DJI_Control_Method_TORQUE):
    {
        PID_Calculate();
        Out = Target_Torque;

        break;
    }
    case (Motor_DJI_Control_Method_OMEGA):
    {
        Omega_ADRC.Init(Rx_Data.Now_Omega);
        Out = Omega_ADRC.Update(Target_Omega + Feedforward_Omega, Rx_Data.Now_Omega);

        break;
    }
    case (Motor_DJI_Control_Method_ANGLE):
    {
        if (Angle_Mode == Motor_DJI_LESO_Angle_Mode_LESO_2ND)
        {
            Position_ADRC.Init(Rx_Data.Now_Angle);
            Out = Position_ADRC.Update(Target_Angle, Rx_Data.Now_Angle, 0.0f);
        }
        else
        {
            PID_Calculate();
            Out = Target_Torque;
        }

        break;
    }
    default:
    {
        Out = 0.0f;

        break;
    }
    }

    Basic_Math_Constrain(&Out, -OUT_MAX, OUT_MAX);

    Output();

    Feedforward_Torque = 0.0f;
    Feedforward_Omega = 0.0f;
}


/**
 * @brief 重置C610的LESO内部状态
 *
 */
void Class_Motor_DJI_C610_LESO::Reset_LESO()
{
    Omega_ADRC = FirstOrderSystemADRC(LESO_Frequency, LESO_Dt, LESO_B, 0.0f, Omega_Current_Max);
    Omega_ADRC.Set_Kp(Omega_Kp);

    Position_ADRC = SecondOrderSystemADRC(LESO_Frequency, LESO_Dt, LESO_B, 0.0f, Position_Current_Max);
    Position_ADRC.Set_Kp(Position_Kp);
    Position_ADRC.Set_Kd(Position_Kd);
}

/**
 * @brief 构造C620 LESO电机对象
 *
 * @param __LESO_Frequency LESO带宽
 * @param __LESO_Dt LESO采样周期
 * @param __LESO_B LESO输入增益
 */
Class_Motor_DJI_C620_LESO::Class_Motor_DJI_C620_LESO(float __LESO_Frequency, float __LESO_Dt, float __LESO_B):
    LESO_Frequency(__LESO_Frequency),
    LESO_Dt(__LESO_Dt),
    LESO_B(__LESO_B),
    Omega_Kp(0.0f),
    Omega_Current_Max(16384.0f),
    Position_Current_Max(16384.0f),
    Position_Kp(0.0f),
    Position_Kd(0.0f),
    Omega_ADRC(__LESO_Frequency, __LESO_Dt, __LESO_B, 0.0f, 16384.0f),
    Position_ADRC(__LESO_Frequency, __LESO_Dt, __LESO_B, 0.0f, 16384.0f)
{
}

/**
 * @brief 100ms存活检测回调, 电机失联时重置LESO
 *
 */
void Class_Motor_DJI_C620_LESO::TIM_100ms_Alive_PeriodElapsedCallback()
{
    Class_Motor_DJI_C620::TIM_100ms_Alive_PeriodElapsedCallback();

    if (Motor_DJI_Status == Motor_DJI_Status_DISABLE)
    {
        Reset_LESO();
    }
}

/**
 * @brief C620计算回调, 输出16bit电流刻度并叠加LESO补偿
 *
 */
void Class_Motor_DJI_C620_LESO::TIM_Calculate_PeriodElapsedCallback()
{
    switch (Motor_DJI_Control_Method)
    {
    case (Motor_DJI_Control_Method_TORQUE):
    {
        PID_Calculate();
        Out = Target_Torque;

        break;
    }
    case (Motor_DJI_Control_Method_OMEGA):
    {
        Omega_ADRC.Init(Rx_Data.Now_Omega);
        Out = Omega_ADRC.Update(Target_Omega + Feedforward_Omega, Rx_Data.Now_Omega);

        break;
    }
    case (Motor_DJI_Control_Method_ANGLE):
    {
        if (Angle_Mode == Motor_DJI_LESO_Angle_Mode_LESO_2ND)
        {
            Position_ADRC.Init(Rx_Data.Now_Angle);
            Out = Position_ADRC.Update(Target_Angle, Rx_Data.Now_Angle, 0.0f);
        }
        else if (Angle_Mode == Motor_DJI_LESO_Angle_Mode_MIX)
        {
            PID_Calculate();
            Position_ADRC.Init(Rx_Data.Now_Angle);
            Out = Position_ADRC.Update(Target_Angle, Rx_Data.Now_Angle, 0.0f) + Target_Torque;
        }
        else
        {
            PID_Calculate();
            Out = Target_Torque;
        }

        break;
    }
    default:
    {
        Out = 0.0f;

        break;
    }
    }

    Basic_Math_Constrain(&Out, -OUT_MAX, OUT_MAX);

    Output();

    Feedforward_Torque = 0.0f;
    Feedforward_Omega = 0.0f;
}


/**
 * @brief 重置C620的LESO内部状态
 *
 */
void Class_Motor_DJI_C620_LESO::Reset_LESO()
{
    Omega_ADRC = FirstOrderSystemADRC(LESO_Frequency, LESO_Dt, LESO_B, 0.0f, Omega_Current_Max);
    Omega_ADRC.Set_Kp(Omega_Kp);

    Position_ADRC = SecondOrderSystemADRC(LESO_Frequency, LESO_Dt, LESO_B, 0.0f, Position_Current_Max);
    Position_ADRC.Set_Kp(Position_Kp);
    Position_ADRC.Set_Kd(Position_Kd);
}
