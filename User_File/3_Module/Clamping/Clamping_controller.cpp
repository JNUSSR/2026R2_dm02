#include "Clamping_controller.h"

#include "Cylinder.h"

ClampingController::ClampingController()
    : angle_planner_(CLAMPING_TARGET_ANGLE_RESET_RAD)
{
}

void ClampingController::PlanToAngle(float target_angle)
{
    angle_planner_.Plan(angle_planner_.GetCurrentTarget(), target_angle, CLAMPING_MOVE_DURATION_S);
}

void ClampingController::Init(FDCAN_HandleTypeDef *hcan)
{
    motor_clamp_2006_.PID_Omega.Init(
        CLAMPING_PID_OMEGA_KP,
        CLAMPING_PID_OMEGA_KI,
        0.0f,
        0.0f,
        CLAMPING_PID_OMEGA_OUT_MAX,
        CLAMPING_PID_OMEGA_I_OUT_MAX);
    motor_clamp_2006_.PID_Angle.Init(
        CLAMPING_PID_ANGLE_KP,
        CLAMPING_PID_ANGLE_KI,
        0.0f,
        0.0f,
        CLAMPING_PID_ANGLE_OUT_MAX,
        CLAMPING_PID_ANGLE_I_OUT_MAX);

    motor_clamp_2006_.Init(hcan, CLAMPING_MOTOR_CAN_ID, Motor_DJI_Control_Method_ANGLE);
    motor_clamp_2006_.Set_Target_Angle(CLAMPING_TARGET_ANGLE_RESET_RAD);
    angle_planner_.ForceSetPosition(CLAMPING_TARGET_ANGLE_RESET_RAD);

    Cylinder_Init();
}

void ClampingController::TaskEntry1ms(void)
{
    motor_clamp_2006_.Set_Target_Angle(angle_planner_.GetNextPosition(CLAMPING_PLAN_DT_S));
    motor_clamp_2006_.TIM_Calculate_PeriodElapsedCallback();
}

void ClampingController::CAN_RxCallback(uint32_t std_id, uint8_t *data)
{
    if (std_id == CLAMPING_MOTOR_STD_ID)
    {
        motor_clamp_2006_.CAN_RxCpltCallback();
    }
}

void ClampingController::MoveToClampAngle(void)
{
    PlanToAngle(CLAMPING_TARGET_ANGLE_CLAMP_RAD);
}

void ClampingController::MoveToResetAngle(void)
{
    PlanToAngle(CLAMPING_TARGET_ANGLE_RESET_RAD);
}

void ClampingController::OpenSolenoid(void)
{
    Cylinder1_Push();
}

void ClampingController::ReleaseSolenoid(void)
{
    Cylinder1_Pull();
}

Class_Motor_DJI_C610 &ClampingController::GetMotor(void)
{
    return motor_clamp_2006_;
}
