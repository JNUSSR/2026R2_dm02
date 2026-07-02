#ifndef R2_USER_MODULE_CLAMPING_CLAMPING_CONTROLLER_H
#define R2_USER_MODULE_CLAMPING_CLAMPING_CONTROLLER_H

#include "stm32h7xx_hal.h"
#include "dvc_motor_dji.h"
#include "dvc_motor_dji_leso.h"
#include "arm.h"

#define CLAMPING_MOTOR_CAN_ID               Motor_DJI_ID_0x202
#define CLAMPING_MOTOR_STD_ID               (0x202U)
#define CLAMPING_TARGET_ANGLE_AIRTAG_RAD    ((PI / 4.0f) * 1.10f)
#define CLAMPING_TARGET_ANGLE_CLAMP_RAD     ((PI / 2.0f) * 1.00f)
#define CLAMPING_TARGET_ANGLE_DOCK_RAD      ((PI)        * 0.95f)
#define CLAMPING_TARGET_ANGLE_RESET_RAD     (0.0f)
#define CLAMPING_PLAN_DT_S                  (0.001f)
#define CLAMPING_MOVE_DURATION_S            (0.5f)

#define CLAMPING_PID_OMEGA_KP           (2300.0f)
#define CLAMPING_PID_OMEGA_KI           (800.0f)
#define CLAMPING_PID_ANGLE_KP           (16.0f)
#define CLAMPING_PID_ANGLE_KI           (0.0f)
#define CLAMPING_PID_OMEGA_I_OUT_MAX    (6500.0f)
#define CLAMPING_PID_OMEGA_OUT_MAX      (8500.0f)
#define CLAMPING_PID_ANGLE_I_OUT_MAX    (10.0f)
#define CLAMPING_PID_ANGLE_OUT_MAX      (15.0f)


#define CLAMPING_LESO_ANGLE_FREQ        (8.0f)
#define CLAMPING_LESO_ANGLE_GAIN        (0.04f)
#define CLAMPING_LESO_ANGLE_KP          (6.0f)
#define CLAMPING_LESO_ANGLE_KD          (0.0f)

class ClampingController
{
private:
    Class_Motor_DJI_C610_LESO motor_clamp_2006_;
    QuinticPlanner angle_planner_;

    void PlanToAngle(float target_angle);

public:
    ClampingController();

    void Init(FDCAN_HandleTypeDef *hcan);
    void TaskEntry1ms(void);
    void CAN_RxCallback(uint32_t std_id, uint8_t *data);

    void MoveToClampAngle(void);
    void MoveToResetAngle(void);
    void MoveToDockAngle(void);
    void MoveToAirtag(void);

    void MoveToAngle(float angle);

    void OpenSolenoid(void);
    void ReleaseSolenoid(void);

    Class_Motor_DJI_C610_LESO &GetMotor(void);
};

#endif
