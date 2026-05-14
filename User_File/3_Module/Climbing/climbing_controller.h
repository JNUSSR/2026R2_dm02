#ifndef TEST_FEEDBACK_CLIMBING_CONTROLLER_H
#define TEST_FEEDBACK_CLIMBING_CONTROLLER_H

#include "stm32h7xx_hal.h"
#include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"
#include "alg_slope.h"
#include "arm.h"

#define TASK_FREQ_HZ (1000.0f)
#define MM_TO_M(x) ((x) / 1000.0f)
#define TURNS_TO_RADS(x) ((x) * 2.0f * PI)

#define SCREW_LEAD_MM_PER_TURN (135.0f)
#define movingmm(x_mm) TURNS_TO_RADS((x_mm) / SCREW_LEAD_MM_PER_TURN)

#define FRONT_LEG_DIR (-1.0f)
#define REAR_LEG_DIR (1.0f)
#define movingmm_front(x_mm) movingmm((x_mm) * FRONT_LEG_DIR)
#define movingmm_rear(x_mm) movingmm((x_mm) * REAR_LEG_DIR)

#define RC_RPM_TO_RADPS(x) ((x) * 2.0f * PI / 60.0f)

#define POS_FRONT_Init movingmm_front(-220.0f)
#define POS_REAR_Init movingmm_rear(-80.0f)

#define POS_FRONT_RETRACT_20cm movingmm_front(-220.0f)
#define POS_REAR_RETRACT_20cm movingmm_rear(-80.0f)
#define POS_FRONT_TOUCH_20cm movingmm_front(-200.0f)
#define POS_REAR_TOUCH_20cm movingmm_rear(0.0f)
#define POS_FRONT_LIFT_20cm movingmm_front(20.0f)
#define POS_REAR_LIFT_20cm movingmm_rear(220.0f)
#define POS_FRONT_FINAL_20cm movingmm_front(-220.0f)
#define POS_REAR_FINAL_20cm movingmm_rear(-80.0f)

#define POS_FRONT_RETRACT_40cm movingmm_front(-420.0f)
#define POS_REAR_RETRACT_40cm movingmm_rear(-75.0f)
#define POS_FRONT_TOUCH_40cm movingmm_front(-400.0f)
#define POS_REAR_TOUCH_40cm movingmm_rear(0.0f)
#define POS_FRONT_LIFT_40cm movingmm_front(20.0f)
#define POS_REAR_LIFT_40cm movingmm_rear(413.0f)
#define POS_FRONT_FINAL_40cm movingmm_front(-220.0f)
#define POS_REAR_FINAL_40cm movingmm_rear(-80.0f)

#define DESCEND_FRONT_TOUCH_TARGET movingmm_front(200.0f)
#define DESCEND_REAR_TOUCH_TARGET movingmm_rear(0.0f)
#define DESCEND_FRONT_GLOBAL_DOWN_TARGET movingmm_front(240.0f)
#define DESCEND_REAR_GLOBAL_DOWN_TARGET movingmm_rear(40.0f)
#define DESCEND_FRONT_RAISE_TARGET movingmm_front(0.0f)
#define DESCEND_REAR_RAISE_TARGET movingmm_rear(-210.0f)

#define WHEEL_RADIUS_M MM_TO_M(50.0f)
#define WHEEL_TRAVEL_UP_M (0.63f)
#define WHEEL_TRAVEL_UP_RAD (WHEEL_TRAVEL_UP_M / WHEEL_RADIUS_M)
#define WHEEL_TRAVEL_DESCEND_M (0.63f)
#define WHEEL_TRAVEL_DESCEND_RAD (WHEEL_TRAVEL_DESCEND_M / WHEEL_RADIUS_M)

#define WHEEL_SLOPE_RPM_UP (85.0f)
#define WHEEL_SLOPE_RPM_DESCEND (70.0f)
#define WHEEL_SLOPE_STEP_UP (RC_RPM_TO_RADPS(WHEEL_SLOPE_RPM_UP) / TASK_FREQ_HZ)
#define WHEEL_SLOPE_STEP_DESCEND (RC_RPM_TO_RADPS(WHEEL_SLOPE_RPM_DESCEND) / TASK_FREQ_HZ)

#define PID_WHEEL_OMEGA_KP (450.0f)
#define PID_WHEEL_OMEGA_KI (200.0f)
#define PID_WHEEL_ANGLE_KP (10.0f)
#define PID_WHEEL_ANGLE_KI (0.0f)
#define WHEEL_CREEP_OMEGA_RADPS (0.3f)

#define SETUP_TEST_REAR_DIR (1.0f)
#define SETUP_TEST_REAR_COMP (2200.0f)

#define PID_FRONT_OMEGA_KP_NORMAL (138.5f)
#define PID_FRONT_ANGLE_KP_NORMAL (70.8f)
#define PID_REAR_OMEGA_KP_NORMAL (167.0f)
#define PID_REAR_ANGLE_KP_NORMAL (85.0f)
#define PID_REAR_OMEGA_KI_NORMAL (10.0f)
#define PID_REAR_ANGLE_KI_NORMAL (1.0f)

#define PID_FRONT_OMEGA_KP_LIFT (305.0f)
#define PID_FRONT_ANGLE_KP_LIFT (140.0f)
#define PID_REAR_OMEGA_KP_LIFT (345.0f)
#define PID_REAR_ANGLE_KP_LIFT (115.0f)
#define PID_REAR_OMEGA_KI_LIFT (10.0f)
#define PID_REAR_ANGLE_KI_LIFT (1.0f)

#define TIME_SETUP 1500
#define TIME_TOUCH 500
#define TIME_LIFT 1500
#define TIME_LIFT_REAR_DELAY 100
#define TIME_DRIVE 2000
#define TIME_RETRACT 1500

#define TIME_DESC_SETUP 1500
#define TIME_DESC_TOUCH 1500
#define TIME_DESC_GLOBAL_DOWN 1000
#define DESCEND_DRIVE_TIME_MS 2000
#define TIME_DESC_RAISE 2000

typedef enum {
    STEP_IDLE = 0,
    STEP_SETUP,
    STEP_TOUCH_DOWN,
    STEP_GLOBAL_LIFT,
    STEP_DRIVE_FWD,
    STEP_RETRACT,
    STEP_DONE,
    STEP_DESCEND_SETUP,
    STEP_DESCEND_TOUCH,
    STEP_DESCEND_GLOBAL_DOWN,
    STEP_DESCEND_DRIVE,
    STEP_DESCEND_RAISE
} ClimbingState_e;

typedef enum {
    CLIMB_UP_MODE_20CM = 0,
    CLIMB_UP_MODE_40CM
} ClimbUpMode_e;

class ClimbingController {
private:
    ClimbingState_e climb_state_;
    uint32_t state_tick_;

    Class_Motor_DJI_C620 motor_lift_front_;
    Class_Motor_DJI_C620 motor_lift_rear_;
    Class_Motor_DJI_C620 motor_wheel_l_;
    Class_Motor_DJI_C620 motor_wheel_r_;

    QuinticPlanner planner_front_pos_{0.0f};
    QuinticPlanner planner_rear_pos_{0.0f};
    Class_Slope slope_wheel_l_angle_;
    Class_Slope slope_wheel_r_angle_;

    uint8_t is_zero_recorded_;
    float start_pos_front_;
    float start_pos_rear_;
    float desc_start_pos_front_;
    float desc_start_pos_rear_;
    float wheel_target_angle_l_;
    float wheel_target_angle_r_;
    uint8_t is_lift_pid_mode_;
    ClimbingState_e prev_climb_state_;
    uint8_t rear_lift_delayed_flag_;

    uint8_t auto_running_;
    uint32_t auto_state_enter_tick_;
    uint8_t descend_mode_;
    uint8_t chassis_external_control_;
    uint8_t init_pose_active_;
    uint8_t init_pose_planned_;
    ClimbUpMode_e up_mode_;

    void HandleStateTransition(uint32_t current_time, uint8_t state_changed);
    void RecordSoftZero(void);
    void UpdatePidAndSlopeByState(void);
    void UpdateStateTargets(uint32_t current_time);
    void RunLiftAndWheelControl(void);
    void SetLiftPidMode(uint8_t enable_lift);
    void SetWheelSlopeStep(float wheel_step);
    bool TryTimeTransition(uint32_t now, uint32_t delay_ms, ClimbingState_e next_state);

public:
    ClimbingController();

    float GetFrontTargetAngle(void) { return motor_lift_front_.Get_Target_Angle(); }
    float GetFrontNowAngle(void) { return motor_lift_front_.Get_Now_Angle(); }
    float GetFrontOut(void) { return motor_lift_front_.Get_Out(); }

    float GetRearTargetAngle(void) { return motor_lift_rear_.Get_Target_Angle(); }
    float GetRearNowAngle(void) { return motor_lift_rear_.Get_Now_Angle(); }
    float GetRearOut(void) { return motor_lift_rear_.Get_Out(); }

    float GetWheelLTargetAngle(void) { return motor_wheel_l_.Get_Target_Angle(); }
    float GetWheelLNowAngle(void) { return motor_wheel_l_.Get_Now_Angle(); }
    float GetWheelRTargetAngle(void) { return motor_wheel_r_.Get_Target_Angle(); }
    float GetWheelRNowAngle(void) { return motor_wheel_r_.Get_Now_Angle(); }

    ClimbingState_e GetState(void) const;
    uint8_t IsAutoRunning(void) const;

    void Init(FDCAN_HandleTypeDef *hcan);
    void TaskEntry1ms(void);
    void AutoTask1ms(void);
    void CAN_RxCallback(uint32_t std_id, uint8_t *data);

    void NextStep(void);
    void AutoStart(void);
    void AutoStart20cm(void);
    void AutoStart40cm(void);
    void Prepare40cm(void);
    void AutoStartFromTouch20cm(void);
    void AutoStartFromTouch40cm(void);
    void DescendAutoStart(void);
    void DescendAutoStart20cm(void);
    void InitPoseStart(void);

    void ManualNext(void);
    void DescendManualNext(void);
    void ManualReset(void);
    void ManualGoto(ClimbingState_e state);
    void EmergencyStop(void);
    void SetChassisExternalControl(uint8_t enable);
};

#endif // TEST_FEEDBACK_CLIMBING_CONTROLLER_H
