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
    Position_Kp(0.0f),
    Position_Kd(0.0f),
    LESO_1st(__LESO_Frequency, __LESO_Dt, __LESO_B),
    LESO_2nd(__LESO_Frequency, __LESO_Dt, __LESO_B)
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
        LESO_1st.Init(Rx_Data.Now_Omega);
        float omega_output = Omega_Kp * (Target_Omega + Feedforward_Omega - Rx_Data.Now_Omega);
        Out = (omega_output - LESO_1st.Get_x2()) / LESO_1st.Get_b();
        LESO_1st.Update(Rx_Data.Now_Omega, Out);
        Basic_Math_Constrain(&Out, -Omega_Current_Max, Omega_Current_Max);

        break;
    }
    case (Motor_DJI_Control_Method_ANGLE):
    {
        if (Angle_Mode == Motor_DJI_LESO_Angle_Mode_LESO_2ND)
        {
            LESO_2nd.Init(Rx_Data.Now_Angle);

            float position_output = Position_Kp * (Target_Angle - Rx_Data.Now_Angle) - Position_Kd * LESO_2nd.Get_x2();
            Out = (position_output - LESO_2nd.Get_x3()) / LESO_2nd.Get_b();
            LESO_2nd.Update(Rx_Data.Now_Angle, Out);
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
    LESO_1st = FirstOrderSystemESO(LESO_Frequency, LESO_Dt, LESO_B);
    LESO_2nd = SecondOrderSystemESO(LESO_Frequency, LESO_Dt, LESO_B);
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
    Position_Kp(0.0f),
    Position_Kd(0.0f),
    LESO_1st(__LESO_Frequency, __LESO_Dt, __LESO_B),
    LESO_2nd(__LESO_Frequency, __LESO_Dt, __LESO_B)
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
        LESO_1st.Init(Rx_Data.Now_Omega);
        float omega_output = Omega_Kp * (Target_Omega + Feedforward_Omega - Rx_Data.Now_Omega);
        Out = (omega_output - LESO_1st.Get_x2()) / LESO_1st.Get_b();
        LESO_1st.Update(Rx_Data.Now_Omega, Out);
        Basic_Math_Constrain(&Out, -Omega_Current_Max, Omega_Current_Max);

        break;
    }
    case (Motor_DJI_Control_Method_ANGLE):
    {
        if (Angle_Mode == Motor_DJI_LESO_Angle_Mode_LESO_2ND)
        {
            LESO_2nd.Init(Rx_Data.Now_Angle);

            float position_output = Position_Kp * (Target_Angle - Rx_Data.Now_Angle) - Position_Kd * LESO_2nd.Get_x2();
            Out = (position_output - LESO_2nd.Get_x3()) / LESO_2nd.Get_b();
            LESO_2nd.Update(Rx_Data.Now_Angle, Out);
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
    LESO_1st = FirstOrderSystemESO(LESO_Frequency, LESO_Dt, LESO_B);
    LESO_2nd = SecondOrderSystemESO(LESO_Frequency, LESO_Dt, LESO_B);
}
