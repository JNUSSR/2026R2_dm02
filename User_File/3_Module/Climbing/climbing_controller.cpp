#include "climbing_controller.h"
#include "fdcan.h"
#include "ChassisTask.h"   

// 剧本 1: 上 20cm 台阶完整流程
static const ActionFrame_t Seq_Up20cm[] = {
    // state_id, front_offset, rear_offset, wheel_rad, duration, delay, PID, wheel_mode, desc_slope
    { STEP_SETUP,               POS_FRONT_RETRACT_20cm, 
          POS_REAR_RETRACT_20cm, 0.0f, 
          TIME_SETUP,            0,   0,
          WHEEL_MODE_ANGLE,       0 },
    { STEP_CHASSIS_APPROACH,  POS_FRONT_RETRACT_20cm, POS_REAR_RETRACT_20cm, 0.0f, TIME_CHASSIS_APPROACH, 0, 0, WHEEL_MODE_CHASSIS_APPROACH, 0 },
    { STEP_TOUCH_DOWN,  POS_FRONT_TOUCH_20cm, POS_REAR_TOUCH_20cm, 0.0f, TIME_TOUCH, 0, 0, WHEEL_MODE_ANGLE, 0 },
    { STEP_GLOBAL_LIFT,  POS_FRONT_LIFT_20cm, POS_REAR_LIFT_20cm, 0.0f, TIME_LIFT, TIME_LIFT_REAR_DELAY, 1, WHEEL_MODE_CREEP, 0 },
    { STEP_DRIVE_FWD,  POS_FRONT_LIFT_20cm, POS_REAR_LIFT_20cm, WHEEL_TRAVEL_UP_RAD, TIME_DRIVE, 0, 1, WHEEL_MODE_ANGLE, 0 },
    { STEP_RETRACT,  POS_FRONT_FINAL_20cm, POS_REAR_FINAL_20cm, 0.0f, TIME_RETRACT, 0, 0, WHEEL_MODE_ANGLE, 0 }
};

// 剧本 2: 上 40cm 台阶完整流程
static const ActionFrame_t Seq_Up40cm[] = {
    { STEP_SETUP, POS_FRONT_RETRACT_40cm, POS_REAR_RETRACT_40cm, 0.0f, TIME_SETUP, 0, 0, WHEEL_MODE_ANGLE, 0 },
    { STEP_CHASSIS_APPROACH, POS_FRONT_RETRACT_40cm, POS_REAR_RETRACT_40cm, 0.0f, TIME_CHASSIS_APPROACH, 0, 0, WHEEL_MODE_CHASSIS_APPROACH, 0 },
    { STEP_TOUCH_DOWN, POS_FRONT_TOUCH_40cm, POS_REAR_TOUCH_40cm, 0.0f, TIME_TOUCH, 0, 0, WHEEL_MODE_ANGLE, 0 },
    { STEP_GLOBAL_LIFT, POS_FRONT_LIFT_40cm, POS_REAR_LIFT_40cm, 0.0f, 3000, TIME_LIFT_REAR_DELAY, 1, WHEEL_MODE_CREEP, 0 },
    { STEP_DRIVE_FWD, POS_FRONT_LIFT_40cm, POS_REAR_LIFT_40cm, WHEEL_TRAVEL_UP_RAD, TIME_DRIVE, 0, 1, WHEEL_MODE_ANGLE, 0 },
    { STEP_RETRACT, POS_FRONT_FINAL_40cm, POS_REAR_FINAL_40cm, 0.0f, TIME_RETRACT, 0, 0, WHEEL_MODE_ANGLE, 0 }
};

// 剧本 3: 下台阶流程 
static const ActionFrame_t Seq_Descend[] = {
    { STEP_DESCEND_FIND_EDGE, POS_FRONT_Init, POS_REAR_Init, 0.0f, TIME_FIND_EDGE_TIMEOUT, 0, 0, WHEEL_MODE_FIND_EDGE, 1 },
    { STEP_DESCEND_SETUP, POS_FRONT_Init, POS_REAR_Init, 0.0f, TIME_DESC_SETUP, 0, 0, WHEEL_MODE_ANGLE, 1 },
    { STEP_DESCEND_TOUCH, DESCEND_FRONT_TOUCH_TARGET, DESCEND_REAR_TOUCH_TARGET, 0.0f, TIME_DESC_TOUCH, 0, 1, WHEEL_MODE_ANGLE, 1 },
    { STEP_DESCEND_GLOBAL_DOWN, DESCEND_FRONT_GLOBAL_DOWN_TARGET, DESCEND_REAR_GLOBAL_DOWN_TARGET, 0.0f, TIME_DESC_GLOBAL_DOWN, 0, 1, WHEEL_MODE_ANGLE, 1 },
    { STEP_DESCEND_DRIVE, DESCEND_FRONT_GLOBAL_DOWN_TARGET, DESCEND_REAR_GLOBAL_DOWN_TARGET, WHEEL_TRAVEL_DESCEND_RAD, DESCEND_DRIVE_TIME_MS, 0, 1, WHEEL_MODE_ANGLE, 1 },
    { STEP_DESCEND_RAISE, DESCEND_FRONT_RAISE_TARGET, DESCEND_REAR_RAISE_TARGET, 0.0f, TIME_DESC_RAISE, 0, 1, WHEEL_MODE_ANGLE, 1 }
};

// 剧本 4: 下40cm台阶流程，由于机械结构，需要后腿在前面先下
static const ActionFrame_t Seq_Descend40cm[] = {
    // state_id, front_offset, rear_offset, wheel_rad, duration, delay, PID, wheel_mode, desc_slope
    { STEP_DESCEND_FIND_EDGE, POS_FRONT_Init, POS_REAR_Init, 0.0f, TIME_FIND_EDGE_TIMEOUT, 0, 0, WHEEL_MODE_FIND_EDGE_BACKWARD, 1 },
    { STEP_DESCEND_SETUP, POS_FRONT_Init, POS_REAR_Init, 
          0.0f, TIME_DESC_SETUP, 0, 0, WHEEL_MODE_ANGLE, 1 },
    { STEP_DESCEND_TOUCH, DESCEND_FRONT_TOUCH_TARGET_40cm, DESCEND_REAR_TOUCH_TARGET_40cm, 
          0.0f, TIME_DESC_TOUCH, 0, 1, WHEEL_MODE_ANGLE, 1 },
    { STEP_DESCEND_GLOBAL_DOWN, DESCEND_FRONT_GLOBAL_DOWN_TARGET_40cm, DESCEND_REAR_GLOBAL_DOWN_TARGET_40cm, 
          0.0f, TIME_DESC_GLOBAL_DOWN, 0, 1, WHEEL_MODE_ANGLE, 1 },
    { STEP_DESCEND_DRIVE, DESCEND_FRONT_GLOBAL_DOWN_TARGET_40cm, DESCEND_REAR_GLOBAL_DOWN_TARGET_40cm, 
          -WHEEL_TRAVEL_DESCEND_RAD, DESCEND_DRIVE_TIME_MS, 0, 1, WHEEL_MODE_ANGLE, 1 },
    { STEP_DESCEND_RAISE, DESCEND_FRONT_RAISE_TARGET_40cm, DESCEND_REAR_RAISE_TARGET_40cm, 
          0.0f, 4000, 0, 1, WHEEL_MODE_ANGLE, 1 }
};


// 单步独立剧本
static const ActionFrame_t Seq_InitPose[] = {
    { STEP_IDLE, POS_FRONT_Init, POS_REAR_Init, 
        0.0f, 2000, 0,
         0, WHEEL_MODE_ANGLE, 0 }
};
static const ActionFrame_t Seq_WeaponHeadClamp[] = {
    { STEP_WEAPON_HEAD_CLAMP, 
          POS_FRONT_WEAPON_HEAD, 
          POS_REAR_Init, 
          0.0f, 
          TIME_WEAPON_ACTION, 
          0, 
          0, 
          WHEEL_MODE_ANGLE, 
          0 }
};
static const ActionFrame_t Seq_WeaponRodDock[] = {
    { STEP_WEAPON_ROD_DOCK, POS_FRONT_WEAPON_ROD, POS_REAR_Init, 0.0f, TIME_WEAPON_ACTION, 0, 0, WHEEL_MODE_ANGLE, 0 }
};

ClimbingController::ClimbingController()
    : current_seq_(nullptr), 
      current_step_(0), 
      total_steps_(0),
      climb_state_(STEP_IDLE),              // 状态机初始为 IDLE
      state_tick_(0),                       // 状态进入时间戳  
      auto_running_(0),
      is_zero_recorded_(0),                 // 软零点尚未记录
       is_lift_pid_mode_(0),              // 当前未启用 lift PID
      rear_lift_delayed_flag_(0),
      start_pos_front_(0.0f),               // 上台阶前腿起始角
      start_pos_rear_(0.0f),                // 上台阶后腿起始角
      wheel_target_angle_l_(0.0f),          // 左轮目标角
      wheel_target_angle_r_(0.0f),          // 右轮目标角
      laser_distance_(0.0f),               
      laser_debounce_cnt_(0) {}             
    

void ClimbingController::Init(FDCAN_HandleTypeDef *hcan)
{
    (void) hcan;

    // 初始化顺序:
    // 1) 斜坡规划器 2) 电机对象 3) PID 参数 4) 运行状态变量
    slope_wheel_l_angle_.Init(WHEEL_SLOPE_STEP_UP, WHEEL_SLOPE_STEP_UP, Slope_First_REAL);
    slope_wheel_r_angle_.Init(WHEEL_SLOPE_STEP_UP, WHEEL_SLOPE_STEP_UP, Slope_First_REAL);

    // 同步 R2_Z 当前测试完备的总线/ID 映射:
    // 前腿: CAN2 / 0x206
    // 后腿: CAN1 / 0x202
    // 左轮: CAN1 / 0x203
    // 右轮: CAN1 / 0x204
    motor_lift_front_.Init(&hfdcan2, Motor_DJI_ID_0x206, Motor_DJI_Control_Method_ANGLE);
    motor_lift_rear_.Init(&hfdcan1, Motor_DJI_ID_0x202, Motor_DJI_Control_Method_ANGLE);
    motor_wheel_l_.Init(&hfdcan1, Motor_DJI_ID_0x203, Motor_DJI_Control_Method_ANGLE);
    motor_wheel_r_.Init(&hfdcan1, Motor_DJI_ID_0x204, Motor_DJI_Control_Method_ANGLE);

    motor_lift_front_.PID_Omega.Init(PID_FRONT_OMEGA_KP_NORMAL, 5.0f, 0.0f, 0.0f, 10000.0f, 12000.0f);
    motor_lift_front_.PID_Angle.Init(PID_FRONT_ANGLE_KP_NORMAL, 0.5f, 0.0f, 0.0f, 10000.0f, 12000.0f);

    motor_lift_rear_.PID_Omega.Init(PID_REAR_OMEGA_KP_NORMAL, 5.0f, 0.0f, 0.0f, 12000.0f, 14000.0f);
    motor_lift_rear_.PID_Angle.Init(PID_REAR_ANGLE_KP_NORMAL, 0.5f, 0.0f, 0.0f, 12000.0f, 14000.0f);

    motor_wheel_l_.PID_Omega.Init(PID_WHEEL_OMEGA_KP, PID_WHEEL_OMEGA_KI, 0.0f, 0.0f, 10000.0f, 12000.0f);
    motor_wheel_l_.PID_Angle.Init(PID_WHEEL_ANGLE_KP, PID_WHEEL_ANGLE_KI, 0.0f, 0.0f, 10000.0f, 12000.0f);
    motor_wheel_r_.PID_Omega.Init(PID_WHEEL_OMEGA_KP, PID_WHEEL_OMEGA_KI, 0.0f, 0.0f, 10000.0f, 12000.0f);
    motor_wheel_r_.PID_Angle.Init(PID_WHEEL_ANGLE_KP, PID_WHEEL_ANGLE_KI, 0.0f, 0.0f, 10000.0f, 12000.0f);
}

void ClimbingController::CAN_RxCallback(uint32_t std_id, uint8_t *data)
{
    (void) data;

    // 206/202~204 分别对应: 前腿/后腿/左轮/右轮
    if (std_id == 0x206)
    {
        motor_lift_front_.CAN_RxCpltCallback();
    }
    else if (std_id == 0x202)
    {
        motor_lift_rear_.CAN_RxCpltCallback();
    }
    else if (std_id == 0x203)
    {
        motor_wheel_l_.CAN_RxCpltCallback();
    }
    else if (std_id == 0x204)
    {
        motor_wheel_r_.CAN_RxCpltCallback();
    }
}

void ClimbingController::RecordSoftZero(void)
{
    start_pos_front_ = motor_lift_front_.Get_Now_Angle();
    start_pos_rear_ = motor_lift_rear_.Get_Now_Angle();
    planner_front_pos_.ForceSetPosition(start_pos_front_);
    planner_rear_pos_.ForceSetPosition(start_pos_rear_);
    
    slope_wheel_l_angle_.Set_Now_Real(motor_wheel_l_.Get_Now_Angle());
    slope_wheel_r_angle_.Set_Now_Real(motor_wheel_r_.Get_Now_Angle());
    slope_wheel_l_angle_.Set_Target(motor_wheel_l_.Get_Now_Angle());
    slope_wheel_r_angle_.Set_Target(motor_wheel_r_.Get_Now_Angle());
    slope_wheel_l_angle_.TIM_Calculate_PeriodElapsedCallback();
    slope_wheel_r_angle_.TIM_Calculate_PeriodElapsedCallback();

    is_zero_recorded_ = 1;
}

// ==========================================
// 表驱动核心：统一发车口与帧加载
// ==========================================
void ClimbingController::StartSequence(const ActionFrame_t* seq, uint8_t count, uint8_t start_index)
{
    // 如果在外部下发动作前还没有记录过零点，强制先记录零点
    if (is_zero_recorded_ == 0) {
        RecordSoftZero();
    }
    current_seq_ = seq;
    total_steps_ = count;
    current_step_ = start_index;
    auto_running_ = 1;
    LoadNextFrame();
}

void ClimbingController::LoadNextFrame(void)
{
    if (current_seq_ == nullptr || current_step_ >= total_steps_) return;

    const ActionFrame_t& frame = current_seq_[current_step_];
    
    climb_state_ = frame.state_id;
    state_tick_ = HAL_GetTick();
    rear_lift_delayed_flag_ = 0;

    // 更新轮子目标 (锁死防飞车，或增加位移)
    slope_wheel_l_angle_.Set_Now_Real(motor_wheel_l_.Get_Now_Angle());
    slope_wheel_r_angle_.Set_Now_Real(motor_wheel_r_.Get_Now_Angle());
    wheel_target_angle_l_ = motor_wheel_l_.Get_Now_Angle() + frame.wheel_travel_rad;
    wheel_target_angle_r_ = motor_wheel_r_.Get_Now_Angle() - frame.wheel_travel_rad;

    // 下发五次多项式规划
    float duration_sec = frame.duration_ms / 1000.0f;
    if (duration_sec < 0.001f) duration_sec = 0.001f; // 防除0保护

    planner_front_pos_.Plan(motor_lift_front_.Get_Now_Angle(), start_pos_front_ + frame.front_offset, duration_sec);
    
    if (frame.rear_delay_ms == 0) {
       planner_rear_pos_.Plan(motor_lift_rear_.Get_Now_Angle(), start_pos_rear_ + frame.rear_offset, duration_sec); 
        rear_lift_delayed_flag_ = 1;
    }

    SetLiftPidMode(frame.lift_pid_enable);
    SetWheelSlopeStep(frame.is_descend_slope ? WHEEL_SLOPE_STEP_DESCEND : WHEEL_SLOPE_STEP_UP);
}

void ClimbingController::AutoTask1ms(void)
{
    if (!auto_running_ || current_seq_ == nullptr) return;

    uint32_t now = HAL_GetTick();
    const ActionFrame_t& frame = current_seq_[current_step_];

    // 独立控制：如果当前帧需要接管底盘持续运动
    if (frame.wheel_mode == WHEEL_MODE_CHASSIS_APPROACH) {
        Chassis_Set_Target(0.1f, 0.0f, 0.0f); 
    }

    // 独立控制：下台阶寻崖感知与防抖拦截
    if (frame.wheel_mode == WHEEL_MODE_FIND_EDGE) {
        Chassis_Set_Target(0.1f, 0.0f, 0.0f); // 给底盘下发微小前移速度
        
        // 判定激光距离是否踩空
        if (laser_distance_ > LASER_EDGE_THRESHOLD_20MM) {
            laser_debounce_cnt_++;
            // 连续 N 毫秒达标，确认找到悬崖
            if (laser_debounce_cnt_ >= LASER_DEBOUNCE_MAX) {
                Chassis_Set_Target(0.0f, 0.0f, 0.0f); // [战术急刹车] 瞬间死区钉住！
                laser_debounce_cnt_ = 0;              // 重置防抖
                
                current_step_++; // 强行跳过时间轴，直接切入 SETUP 帧
                LoadNextFrame(); // 下发收腿剧本
                return;          // 拦截本次循环，禁止执行下面的时间判断
            }
        } else {
            // 中途如果有一毫秒读回了正常值（如10cm），说明是噪点，防抖计数器清零
            laser_debounce_cnt_ = 0; 
        }
    }

    if (frame.wheel_mode == WHEEL_MODE_FIND_EDGE_BACKWARD) {
        Chassis_Set_Target(-0.1f, 0.0f, 0.0f); // 给底盘下发向后(负数)的微小速度
        
        if (laser_distance_rear_ > LASER_EDGE_THRESHOLD_40MM) {
            laser_debounce_cnt_rear_++;
            if (laser_debounce_cnt_rear_ >= LASER_DEBOUNCE_MAX) {
                Chassis_Set_Target(0.0f, 0.0f, 0.0f); // 瞬间死区钉住！
                laser_debounce_cnt_rear_ = 0;              
                current_step_++; 
                LoadNextFrame(); 
                return;          
            }
        } else {
            laser_debounce_cnt_rear_ = 0; 
        }
    }

    // 核心流转：如果时间走完，切换下一帧
    if ((now - state_tick_) >= frame.duration_ms)
    {
        // 离开帧前，清空特定状态命令
        if (frame.wheel_mode == WHEEL_MODE_CHASSIS_APPROACH) {
            Chassis_Set_Target(0.0f, 0.0f, 0.0f);
        }

        //如果寻崖模式跑满了 8 秒还没触发上面的跳帧，说明找不着台阶！
        if (frame.wheel_mode == WHEEL_MODE_FIND_EDGE || frame.wheel_mode == WHEEL_MODE_FIND_EDGE_BACKWARD) {
            Chassis_Set_Target(0.0f, 0.0f, 0.0f); // 安全刹车
            auto_running_ = 0;                              // 强制停机，保护机器人不乱跑
            return; 
        }

        current_step_++;
        if (current_step_ >= total_steps_) {
            auto_running_ = 0; // 剧本放映结束，自动维持在最后一个动作
        } else {
            LoadNextFrame();
        }
    }
}

void ClimbingController::TaskEntry1ms(void)
{
    if (is_zero_recorded_ == 0) {
        RecordSoftZero();
    }

    UpdateStateTargets(HAL_GetTick());
    RunLiftAndWheelControl();
}

void ClimbingController::UpdateStateTargets(uint32_t current_time)
{
    if (!auto_running_ || current_seq_ == nullptr) return;
    const ActionFrame_t& frame = current_seq_[current_step_];

    slope_wheel_l_angle_.Set_Target(wheel_target_angle_l_);
    slope_wheel_r_angle_.Set_Target(wheel_target_angle_r_);

    // 蹲守后腿的延时启动
    if (rear_lift_delayed_flag_ == 0)
    {
        if ((current_time - state_tick_) >= frame.rear_delay_ms)
        {
            float time_left_sec = (frame.duration_ms - frame.rear_delay_ms) / 1000.0f;
            if (time_left_sec < 0.001f) time_left_sec = 0.001f;
            
            planner_rear_pos_.Plan(motor_lift_rear_.Get_Now_Angle(), start_pos_rear_ + frame.rear_offset, time_left_sec);
            rear_lift_delayed_flag_ = 1;
        }
    }
}

void ClimbingController::RunLiftAndWheelControl(void)
{
    float target_front = planner_front_pos_.GetNextPosition(0.001f);
    float target_rear  = planner_rear_pos_.GetNextPosition(0.001f);

    motor_lift_front_.Set_Target_Angle(target_front);
    motor_lift_rear_.Set_Target_Angle(target_rear);

    slope_wheel_l_angle_.TIM_Calculate_PeriodElapsedCallback();
    slope_wheel_r_angle_.TIM_Calculate_PeriodElapsedCallback();

    WheelMode_e current_w_mode = WHEEL_MODE_ANGLE;
    if (auto_running_ && current_seq_ != nullptr) {
        current_w_mode = current_seq_[current_step_].wheel_mode;
    }

    // 根据模式切断/开启 轮子PID 和 下发策略
    if (current_w_mode == WHEEL_MODE_CREEP)
    {
        if (motor_wheel_l_.Get_Control_Method() != Motor_DJI_Control_Method_OMEGA) {
            motor_wheel_l_.Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
            motor_wheel_r_.Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
            motor_wheel_l_.PID_Angle.Set_Integral_Error(0.0f);
            motor_wheel_l_.PID_Omega.Set_Integral_Error(0.0f);
            motor_wheel_r_.PID_Angle.Set_Integral_Error(0.0f);
            motor_wheel_r_.PID_Omega.Set_Integral_Error(0.0f);
        }
        motor_wheel_l_.Set_Target_Omega(WHEEL_CREEP_OMEGA_RADPS);
        motor_wheel_r_.Set_Target_Omega(-WHEEL_CREEP_OMEGA_RADPS);
    }
    else
    {
        if (motor_wheel_l_.Get_Control_Method() != Motor_DJI_Control_Method_ANGLE) {
            motor_wheel_l_.Set_Control_Method(Motor_DJI_Control_Method_ANGLE);
            motor_wheel_r_.Set_Control_Method(Motor_DJI_Control_Method_ANGLE);
            
            // 切回位置控制时，保留此前蠕动的Omega扭矩
            motor_wheel_l_.PID_Angle.Set_Integral_Error(0.0f);
            motor_wheel_r_.PID_Angle.Set_Integral_Error(0.0f);
        }
        motor_wheel_l_.Set_Target_Angle(slope_wheel_l_angle_.Get_Out());
        motor_wheel_r_.Set_Target_Angle(slope_wheel_r_angle_.Get_Out());
    }

    motor_lift_front_.TIM_Calculate_PeriodElapsedCallback();
    motor_lift_rear_.TIM_Calculate_PeriodElapsedCallback();
    motor_wheel_l_.TIM_Calculate_PeriodElapsedCallback();
    motor_wheel_r_.TIM_Calculate_PeriodElapsedCallback();
}

// 辅助方法：动态 PID
void ClimbingController::SetLiftPidMode(uint8_t enable_lift)
{
    if (is_lift_pid_mode_ == enable_lift) return; 
    if (enable_lift) {
        motor_lift_front_.PID_Omega.Set_K_P(PID_FRONT_OMEGA_KP_LIFT);
        motor_lift_front_.PID_Angle.Set_K_P(PID_FRONT_ANGLE_KP_LIFT);
        motor_lift_rear_.PID_Omega.Set_K_P(PID_REAR_OMEGA_KP_LIFT);
        motor_lift_rear_.PID_Angle.Set_K_P(PID_REAR_ANGLE_KP_LIFT);
        motor_lift_rear_.PID_Omega.Set_K_I(PID_REAR_OMEGA_KI_LIFT);
        motor_lift_rear_.PID_Angle.Set_K_I(PID_REAR_ANGLE_KI_LIFT);
    } else {
        motor_lift_front_.PID_Omega.Set_K_P(PID_FRONT_OMEGA_KP_NORMAL);
        motor_lift_front_.PID_Angle.Set_K_P(PID_FRONT_ANGLE_KP_NORMAL);
        motor_lift_rear_.PID_Omega.Set_K_P(PID_REAR_OMEGA_KP_NORMAL);
        motor_lift_rear_.PID_Angle.Set_K_P(PID_REAR_ANGLE_KP_NORMAL);
        motor_lift_rear_.PID_Omega.Set_K_I(PID_REAR_OMEGA_KI_NORMAL);
        motor_lift_rear_.PID_Angle.Set_K_I(PID_REAR_ANGLE_KI_NORMAL);
    }
    is_lift_pid_mode_ = enable_lift;
}

// 辅助方法：轮子斜坡率
void ClimbingController::SetWheelSlopeStep(float wheel_step)
{
    slope_wheel_l_angle_.Set_Increase_Value(wheel_step);
    slope_wheel_l_angle_.Set_Decrease_Value(wheel_step);
    slope_wheel_r_angle_.Set_Increase_Value(wheel_step);
    slope_wheel_r_angle_.Set_Decrease_Value(wheel_step);
}

// ==========================================
// 外部调用接口实现
// ==========================================
void ClimbingController::AutoStart(void) { StartSequence(Seq_Up20cm, 6, 0); }
void ClimbingController::AutoStart20cm(void) { StartSequence(Seq_Up20cm, 6, 0); }
void ClimbingController::AutoStart40cm(void) { StartSequence(Seq_Up40cm, 6, 0); }

void ClimbingController::Prepare40cm(void) { 
    StartSequence(Seq_Up40cm, 6, 0); 
    auto_running_ = 0; // 加载第一帧但不自动推进时间
}

//上20cm接口
void ClimbingController::AutoStartFromTouch20cm(void) { StartSequence(Seq_Up20cm, 6, 1); }
void ClimbingController::AutoStartFromTouch40cm(void) { StartSequence(Seq_Up40cm, 6, 1); }

//下20cm接口
void ClimbingController::DescendAutoStart(void) { StartSequence(Seq_Descend, 6, 0); }
void ClimbingController::DescendAutoStart20cm(void) { StartSequence(Seq_Descend, 6, 0); }

//下40cm接口
void ClimbingController::DescendAutoStart40cm(void) { StartSequence(Seq_Descend40cm, 6, 0); }

//武器头和武器杆
void ClimbingController::WeaponHeadClampStart(void) { StartSequence(Seq_WeaponHeadClamp, 1, 0); }
void ClimbingController::WeaponRodDockStart(void) { StartSequence(Seq_WeaponRodDock, 1, 0); }

void ClimbingController::InitPoseStart(void) { StartSequence(Seq_InitPose, 1, 0); }

uint8_t ClimbingController::IsFindingEdgeFront(void) const
{
    if (!auto_running_ || current_seq_ == nullptr) return 0;
    return (current_seq_[current_step_].wheel_mode == WHEEL_MODE_FIND_EDGE) ? 1 : 0;
}

uint8_t ClimbingController::IsFindingEdgeRear(void) const
{
    if (!auto_running_ || current_seq_ == nullptr) return 0;
    return (current_seq_[current_step_].wheel_mode == WHEEL_MODE_FIND_EDGE_BACKWARD) ? 1 : 0;
}