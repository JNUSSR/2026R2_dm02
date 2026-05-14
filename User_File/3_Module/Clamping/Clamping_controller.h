#ifndef R2_USER_MODULE_CLAMPING_CLAMPING_CONTROLLER_H
#define R2_USER_MODULE_CLAMPING_CLAMPING_CONTROLLER_H

#include "stm32h7xx_hal.h"
#include "dvc_motor_dji.h"
#include "arm.h"

#define CLAMPING_MOTOR_CAN_ID           Motor_DJI_ID_0x207
#define CLAMPING_MOTOR_STD_ID           (0x207U)
#define CLAMPING_TARGET_ANGLE_CLAMP_RAD (-PI / 2.0f)
#define CLAMPING_TARGET_ANGLE_RESET_RAD (0.0f)
#define CLAMPING_PLAN_DT_S              (0.001f)
#define CLAMPING_MOVE_DURATION_S        (0.3f)

#define CLAMPING_PID_OMEGA_KP           (1800.0f)
#define CLAMPING_PID_OMEGA_KI           (500.0f)
#define CLAMPING_PID_ANGLE_KP           (15.0f)
#define CLAMPING_PID_ANGLE_KI           (0.0f)
#define CLAMPING_PID_OMEGA_OUT_MAX      (4000.0f)
#define CLAMPING_PID_OMEGA_I_OUT_MAX    (6000.0f)
#define CLAMPING_PID_ANGLE_OUT_MAX      (10.0f)
#define CLAMPING_PID_ANGLE_I_OUT_MAX    (15.0f)

class ClampingController
{
private:
    Class_Motor_DJI_C610 motor_clamp_2006_;
    QuinticPlanner angle_planner_;

    void PlanToAngle(float target_angle);

public:
    ClampingController();

    void Init(FDCAN_HandleTypeDef *hcan);
    void TaskEntry1ms(void);
    void CAN_RxCallback(uint32_t std_id, uint8_t *data);

    void MoveToClampAngle(void);
    void MoveToResetAngle(void);

    void OpenSolenoid(void);
    void ReleaseSolenoid(void);

    Class_Motor_DJI_C610 &GetMotor(void);
};

#endif
