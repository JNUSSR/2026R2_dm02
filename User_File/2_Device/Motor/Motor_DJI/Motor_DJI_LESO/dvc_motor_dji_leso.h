/**
 * @file dvc_motor_dji_leso.h
 * @brief DJI电机LESO派生类
 */

#ifndef DVC_MOTOR_DJI_LESO_H
#define DVC_MOTOR_DJI_LESO_H

#include "1_Middleware/Algorithm/ADRC/LESO/LESO.h"
#include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"

enum Enum_Motor_DJI_LESO_Angle_Mode
{
    Motor_DJI_LESO_Angle_Mode_ORIGIN = 0,
    Motor_DJI_LESO_Angle_Mode_LESO_2ND,
};

/**
 * @brief C610 LESO电机派生类
 *
 */
class Class_Motor_DJI_C610_LESO : public Class_Motor_DJI_C610
{
public:
    Class_Motor_DJI_C610_LESO(float __LESO_Frequency = 300.0f, float __LESO_Dt = 0.001f, float __LESO_B = 1.0f);

    void TIM_100ms_Alive_PeriodElapsedCallback();

    void TIM_Calculate_PeriodElapsedCallback();

    void Set_Angle_Mode(Enum_Motor_DJI_LESO_Angle_Mode __Angle_Mode);

    void Set_Position_Kp(float __Position_Kp);

    void Set_Position_Kd(float __Position_Kd);

    void Set_Position_ADRC(float __LESO_Frequency, float __LESO_Dt, float __LESO_B, float __Position_Kp, float __Position_Kd);

    void Set_Omega_Kp(float __Omega_Kp);

    float Get_Omega_Kp();

    void Set_Omega_Current_Max(float __Omega_Current_Max);

    float Get_Omega_Current_Max();

    void Set_Omega_ADRC(float __LESO_Frequency, float __LESO_Dt, float __LESO_B, float __Omega_Kp, float __Omega_Current_Max);

protected:
    float LESO_Frequency;
    float LESO_Dt;
    float LESO_B;
    float Omega_Kp = 0.0f;
    float Omega_Current_Max = 0.0f;
    float Position_Kp = 0.0f;
    float Position_Kd = 0.0f;
    Enum_Motor_DJI_LESO_Angle_Mode Angle_Mode = Motor_DJI_LESO_Angle_Mode_ORIGIN;
    FirstOrderSystemESO LESO_1st;
    SecondOrderSystemESO LESO_2nd;

    void Reset_LESO();
};

/**
 * @brief C620 LESO电机派生类
 *
 */
class Class_Motor_DJI_C620_LESO : public Class_Motor_DJI_C620
{
public:
    Class_Motor_DJI_C620_LESO(float __LESO_Frequency = 40.0f, float __LESO_Dt = 0.001f, float __LESO_B = 0.05f);

    void TIM_100ms_Alive_PeriodElapsedCallback();

    void TIM_Calculate_PeriodElapsedCallback();

    void Set_Angle_Mode(Enum_Motor_DJI_LESO_Angle_Mode __Angle_Mode);

    void Set_Position_Kp(float __Position_Kp);

    void Set_Position_Kd(float __Position_Kd);

    void Set_Position_ADRC(float __LESO_Frequency, float __LESO_Dt, float __LESO_B, float __Position_Kp, float __Position_Kd);

    void Set_Omega_Kp(float __Omega_Kp);

    float Get_Omega_Kp();

    void Set_Omega_Current_Max(float __Omega_Current_Max);

    float Get_Omega_Current_Max();

    void Set_Omega_ADRC(float __LESO_Frequency, float __LESO_Dt, float __LESO_B, float __Omega_Kp, float __Omega_Current_Max);

protected:
    float LESO_Frequency;
    float LESO_Dt;
    float LESO_B;
    float Omega_Kp = 0.0f;
    float Omega_Current_Max = 0.0f;
    float Position_Kp = 0.0f;
    float Position_Kd = 0.0f;
    Enum_Motor_DJI_LESO_Angle_Mode Angle_Mode = Motor_DJI_LESO_Angle_Mode_ORIGIN;
    FirstOrderSystemESO LESO_1st;
    SecondOrderSystemESO LESO_2nd;

    void Reset_LESO();
};
/**
 * @brief 设置ANGLE模式下的控制路径
 *
 * @param __Angle_Mode 原始串级或二阶LESO位置控制
 */
inline void Class_Motor_DJI_C610_LESO::Set_Angle_Mode(Enum_Motor_DJI_LESO_Angle_Mode __Angle_Mode)
{
    Angle_Mode = __Angle_Mode;
}

/**
 * @brief 设置二阶位置控制的比例系数
 *
 * @param __Position_Kp 位置比例系数
 */
inline void Class_Motor_DJI_C610_LESO::Set_Position_Kp(float __Position_Kp)
{
    Position_Kp = __Position_Kp;
}

/**
 * @brief 设置二阶位置控制的速度阻尼系数
 *
 * @param __Position_Kd 位置微分系数
 */
inline void Class_Motor_DJI_C610_LESO::Set_Position_Kd(float __Position_Kd)
{
    Position_Kd = __Position_Kd;
}

/**
 * @brief 一次性设置二阶位置控制参数
 *
 * @param __LESO_Frequency 二阶LESO带宽
 * @param __LESO_Dt 二阶LESO采样周期
 * @param __LESO_B 二阶LESO输入增益
 * @param __Position_Kp 位置比例系数
 * @param __Position_Kd 位置微分系数
 */
inline void Class_Motor_DJI_C610_LESO::Set_Position_ADRC(float __LESO_Frequency, float __LESO_Dt, float __LESO_B, float __Position_Kp, float __Position_Kd)
{
    LESO_Frequency = __LESO_Frequency;
    LESO_Dt = __LESO_Dt;
    LESO_B = __LESO_B;
    Position_Kp = __Position_Kp;
    Position_Kd = __Position_Kd;
    Reset_LESO();
}

/**
 * @brief 设置一阶速度控制的比例系数
 *
 * @param __Omega_Kp 速度比例系数
 */
inline void Class_Motor_DJI_C610_LESO::Set_Omega_Kp(float __Omega_Kp)
{
    Omega_Kp = __Omega_Kp;
}

/**
 * @brief 获取一阶速度控制的比例系数
 *
 * @return float 速度比例系数
 */
inline float Class_Motor_DJI_C610_LESO::Get_Omega_Kp()
{
    return (Omega_Kp);
}

/**
 * @brief 设置一阶速度控制的电流限幅
 *
 * @param __Omega_Current_Max 速度控制电流限幅
 */
inline void Class_Motor_DJI_C610_LESO::Set_Omega_Current_Max(float __Omega_Current_Max)
{
    Omega_Current_Max = __Omega_Current_Max;
}

/**
 * @brief 获取一阶速度控制的电流限幅
 *
 * @return float 速度控制电流限幅
 */
inline float Class_Motor_DJI_C610_LESO::Get_Omega_Current_Max()
{
    return (Omega_Current_Max);
}

/**
 * @brief 一次性设置一阶速度ADRC参数
 *
 * @param __LESO_Frequency 一阶LESO带宽
 * @param __LESO_Dt 一阶LESO采样周期
 * @param __LESO_B 一阶LESO输入增益
 * @param __Omega_Kp 速度比例系数
 * @param __Omega_Current_Max 速度控制电流限幅
 */
inline void Class_Motor_DJI_C610_LESO::Set_Omega_ADRC(float __LESO_Frequency, float __LESO_Dt, float __LESO_B, float __Omega_Kp, float __Omega_Current_Max)
{
    LESO_Frequency = __LESO_Frequency;
    LESO_Dt = __LESO_Dt;
    LESO_B = __LESO_B;
    Omega_Kp = __Omega_Kp;
    Omega_Current_Max = __Omega_Current_Max;
    Reset_LESO();
}

/**
 * @brief 设置ANGLE模式下的控制路径
 *
 * @param __Angle_Mode 原始串级或二阶LESO位置控制
 */
inline void Class_Motor_DJI_C620_LESO::Set_Angle_Mode(Enum_Motor_DJI_LESO_Angle_Mode __Angle_Mode)
{
    Angle_Mode = __Angle_Mode;
}

/**
 * @brief 设置二阶位置控制的比例系数
 *
 * @param __Position_Kp 位置比例系数
 */
inline void Class_Motor_DJI_C620_LESO::Set_Position_Kp(float __Position_Kp)
{
    Position_Kp = __Position_Kp;
}

/**
 * @brief 设置二阶位置控制的速度阻尼系数
 *
 * @param __Position_Kd 位置微分系数
 */
inline void Class_Motor_DJI_C620_LESO::Set_Position_Kd(float __Position_Kd)
{
    Position_Kd = __Position_Kd;
}

/**
 * @brief 一次性设置二阶位置控制参数
 *
 * @param __LESO_Frequency 二阶LESO带宽
 * @param __LESO_Dt 二阶LESO采样周期
 * @param __LESO_B 二阶LESO输入增益
 * @param __Position_Kp 位置比例系数
 * @param __Position_Kd 位置微分系数
 */
inline void Class_Motor_DJI_C620_LESO::Set_Position_ADRC(float __LESO_Frequency, float __LESO_Dt, float __LESO_B, float __Position_Kp, float __Position_Kd)
{
    LESO_Frequency = __LESO_Frequency;
    LESO_Dt = __LESO_Dt;
    LESO_B = __LESO_B;
    Position_Kp = __Position_Kp;
    Position_Kd = __Position_Kd;
    Reset_LESO();
}

/**
 * @brief 设置一阶速度控制的比例系数
 *
 * @param __Omega_Kp 速度比例系数
 */
inline void Class_Motor_DJI_C620_LESO::Set_Omega_Kp(float __Omega_Kp)
{
    Omega_Kp = __Omega_Kp;
}

/**
 * @brief 获取一阶速度控制的比例系数
 *
 * @return float 速度比例系数
 */
inline float Class_Motor_DJI_C620_LESO::Get_Omega_Kp()
{
    return (Omega_Kp);
}

/**
 * @brief 设置一阶速度控制的电流限幅
 *
 * @param __Omega_Current_Max 速度控制电流限幅
 */
inline void Class_Motor_DJI_C620_LESO::Set_Omega_Current_Max(float __Omega_Current_Max)
{
    Omega_Current_Max = __Omega_Current_Max;
}

/**
 * @brief 获取一阶速度控制的电流限幅
 *
 * @return float 速度控制电流限幅
 */
inline float Class_Motor_DJI_C620_LESO::Get_Omega_Current_Max()
{
    return (Omega_Current_Max);
}

/**
 * @brief 一次性设置一阶速度ADRC参数
 *
 * @param __LESO_Frequency 一阶LESO带宽
 * @param __LESO_Dt 一阶LESO采样周期
 * @param __LESO_B 一阶LESO输入增益
 * @param __Omega_Kp 速度比例系数
 * @param __Omega_Current_Max 速度控制电流限幅
 */
inline void Class_Motor_DJI_C620_LESO::Set_Omega_ADRC(float __LESO_Frequency, float __LESO_Dt, float __LESO_B, float __Omega_Kp, float __Omega_Current_Max)
{
    LESO_Frequency = __LESO_Frequency;
    LESO_Dt = __LESO_Dt;
    LESO_B = __LESO_B;
    Omega_Kp = __Omega_Kp;
    Omega_Current_Max = __Omega_Current_Max;
    Reset_LESO();
}

#endif
