#ifndef TEST_FEEDBACK_CLIMBING_CONTROLLER_H
#define TEST_FEEDBACK_CLIMBING_CONTROLLER_H

#include "stm32h7xx_hal.h"
#include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"
#include "alg_slope.h"
#include "arm.h"

// ==========================================
// 0. 基础换算工具
// ==========================================

#define TASK_FREQ_HZ        (1000.0f)
#define MM_TO_M(x)          ((x) / 1000.0f)
#define TURNS_TO_RADS(x)    ((x) * 2.0f * PI)

// 丝杆导程: 每转一圈对应的直线位移(mm)
#define SCREW_LEAD_MM_PER_TURN   (135.0f)
#define movingmm(x_mm)       TURNS_TO_RADS((x_mm) / SCREW_LEAD_MM_PER_TURN)

// 前/后腿方向修正（-1.0f 反向）
#define FRONT_LEG_DIR        (1.0f)
#define REAR_LEG_DIR         (1.0f)
#define movingmm_front(x_mm) movingmm((x_mm) * FRONT_LEG_DIR)
#define movingmm_rear(x_mm)  movingmm((x_mm) * REAR_LEG_DIR)

// 避免和 dvc_motor.h 里的同名宏冲突
#define RC_RPM_TO_RADPS(x)  ((x) * 2.0f * PI / 60.0f)

// ==========================================
// 1. 机械位置参数(上/下台阶)  以下都是负值向上，正值向下
// ==========================================
//初始状态
#define POS_FRONT_Init       movingmm_front(-220.0f)
#define POS_REAR_Init        movingmm_rear(-80.0f)

//上20cm台阶
#define POS_FRONT_RETRACT_20cm    movingmm_front(-220.0f)
#define POS_REAR_RETRACT_20cm     movingmm_rear(-80.0f)

#define POS_FRONT_TOUCH_20cm      movingmm_front(-200.0f) //-200
#define POS_REAR_TOUCH_20cm       movingmm_rear(0.0f)     //0.0

#define POS_FRONT_LIFT_20cm       movingmm_front(20.0f) //20
#define POS_REAR_LIFT_20cm        movingmm_rear(220.0f) //220

#define POS_FRONT_FINAL_20cm      movingmm_front(-220.0f)
#define POS_REAR_FINAL_20cm       movingmm_rear(-80.0f)

//上40cm台阶
#define POS_FRONT_RETRACT_40cm    movingmm_front(-420.0f)
#define POS_REAR_RETRACT_40cm     movingmm_rear(-75.0f)

#define POS_FRONT_TOUCH_40cm      movingmm_front(-400.0f)
#define POS_REAR_TOUCH_40cm       movingmm_rear(0.0f)

#define POS_FRONT_LIFT_40cm       movingmm_front(20.0f)
#define POS_REAR_LIFT_40cm        movingmm_rear(413.0f)

#define POS_FRONT_FINAL_40cm movingmm_front(-220.0f)
#define POS_REAR_FINAL_40cm  movingmm_rear(-80.0f)

//下20cm台阶
// 下台阶：状态3 触地目标
#define DESCEND_FRONT_TOUCH_TARGET     movingmm_front(200.0f)
#define DESCEND_REAR_TOUCH_TARGET      movingmm_rear(0.0f)

// 下台阶：状态4 全局下降目标
#define DESCEND_FRONT_GLOBAL_DOWN_TARGET movingmm_front(240.0f)
#define DESCEND_REAR_GLOBAL_DOWN_TARGET  movingmm_rear(40.0f)

// 下台阶：状态6 抬起过渡目标
// 前脚保持下层地面接触，后脚保持上层台阶对应高度
#define DESCEND_FRONT_RAISE_TARGET       movingmm_front(0.0f)
#define DESCEND_REAR_RAISE_TARGET        movingmm_rear(-210.0f)

//下40cm台阶
// 下台阶：状态3 触地目标
#define DESCEND_FRONT_TOUCH_TARGET_40cm       movingmm_front(0.0f)
#define DESCEND_REAR_TOUCH_TARGET_40cm        movingmm_rear(400.0f)

// 下台阶：状态4 全局下降目标
#define DESCEND_FRONT_GLOBAL_DOWN_TARGET_40cm movingmm_front(40.0f)
#define DESCEND_REAR_GLOBAL_DOWN_TARGET_40cm  movingmm_rear(440.0f)

// 下台阶：状态6 抬起过渡目标
// 前脚保持下层地面接触，后脚保持上层台阶对应高度
#define DESCEND_FRONT_RAISE_TARGET_40cm       movingmm_front(-400.0f)
#define DESCEND_REAR_RAISE_TARGET_40cm        movingmm_rear(0.0f)

// ==========================================
// 武器机构对接位置与时间参数
// ==========================================
// 夹武器头前腿位置 (请根据实际物理高度微调)
#define POS_FRONT_WEAPON_HEAD    movingmm_front(-215.0f) 
// 对接武器杆前腿位置 (请根据实际物理高度微调)
#define POS_FRONT_WEAPON_ROD     movingmm_front(-300.0f) 

// ==========================================
// 2. 速度、斜坡、时间参数
// ==========================================

// 轮子角度模式：距离->角度（rad）
//上台阶
#define WHEEL_RADIUS_M                MM_TO_M(50.0f)
#define WHEEL_TRAVEL_UP_M             (0.63f)
#define WHEEL_TRAVEL_UP_RAD           (WHEEL_TRAVEL_UP_M / WHEEL_RADIUS_M)
//下台阶
#define WHEEL_TRAVEL_DESCEND_M        (0.63f)
#define WHEEL_TRAVEL_DESCEND_RAD      (WHEEL_TRAVEL_DESCEND_M / WHEEL_RADIUS_M)
//#define WHEEL_ANGLE_DONE_TOL_RAD      (0.25f) //轮子角度到位容忍度 即误差到某个值时认为轮子已经到位

// 轮子角度斜坡
#define WHEEL_SLOPE_RPM_UP            (90.0f) //up
#define WHEEL_SLOPE_RPM_DESCEND       (80.0f) //descend
#define WHEEL_SLOPE_STEP_UP           (RC_RPM_TO_RADPS(WHEEL_SLOPE_RPM_UP) / TASK_FREQ_HZ)
#define WHEEL_SLOPE_STEP_DESCEND      (RC_RPM_TO_RADPS(WHEEL_SLOPE_RPM_DESCEND) / TASK_FREQ_HZ)

// 轮子角度模式 PID
#define PID_WHEEL_OMEGA_KP            (450.0f)
#define PID_WHEEL_OMEGA_KI            (200.0f)
#define PID_WHEEL_ANGLE_KP            (10.0f)
#define PID_WHEEL_ANGLE_KI            (0.0f)
#define WHEEL_CREEP_OMEGA_RADPS       (0.3f)

// 动态PID参数（按工况切换）
#define PID_FRONT_OMEGA_KP_NORMAL   (138.5f) //正常前轮P
#define PID_FRONT_ANGLE_KP_NORMAL   (70.8f)
#define PID_REAR_OMEGA_KP_NORMAL    (167.0f) //正常后轮P
#define PID_REAR_ANGLE_KP_NORMAL    (85.0f)
#define PID_REAR_OMEGA_KI_NORMAL    (10.0f) //正常后轮I
#define PID_REAR_ANGLE_KI_NORMAL    (1.0f)

#define PID_FRONT_OMEGA_KP_LIFT     (305.0f) //抬升前轮P
#define PID_FRONT_ANGLE_KP_LIFT     (140.0f)
#define PID_REAR_OMEGA_KP_LIFT      (345.0f) //抬升后轮P
#define PID_REAR_ANGLE_KP_LIFT      (115.0f)
#define PID_REAR_OMEGA_KI_LIFT      (10.0f) //抬升后轮I
#define PID_REAR_ANGLE_KI_LIFT      (1.0f)

// --- 上台阶时间参数 ---
#define TIME_SETUP           1500  // 给2秒让它缩腿
#define TIME_CHASSIS_APPROACH 800  // 底盘向前微调贴紧台阶的时间 (0.8s)
#define TIME_TOUCH           500  // 触地时间
#define TIME_LIFT            1500  // 顶升时间
#define TIME_LIFT_REAR_DELAY 100   // 顶升阶段后脚延时启动
#define TIME_DRIVE           1800  // 平移时间
#define TIME_RETRACT         1000  // 收腿时间

// --- 下台阶时间参数 ---
#define TIME_DESC_SETUP      1500
#define TIME_DESC_TOUCH      1500  // 下台阶：状态3 触地时间
#define TIME_DESC_GLOBAL_DOWN  1000 // 下台阶：状态4 全局下降时间
#define DESCEND_DRIVE_TIME_MS  2000 // 下台阶：状态5 平移时间
#define TIME_DESC_RAISE      2000 // 下台阶：状态6 抬升时间

//夹武器头对应时间参数
#define TIME_WEAPON_ACTION       1500// 武器动作统一执行时间 (1.5秒平滑到位)

// 激光测距寻崖参数
#define LASER_EDGE_THRESHOLD_20MM   620.0f  // 判定门限：激光返回大于 62cm (620mm) 视为踩空
#define LASER_EDGE_THRESHOLD_40MM   820.0f  // 判定门限：激光返回大于 62cm (620mm) 视为踩空
#define LASER_DEBOUNCE_MAX        50       // 防抖：连续 5ms 检测到大于门限才触发
#define TIME_FIND_EDGE_TIMEOUT    8000    // 寻崖超时：最多往前开 8 秒，找不到就停机保护


// ==========================================
// 3. 状态定义
// ==========================================

typedef enum {
    STEP_IDLE = 0,            // 空闲
    STEP_SETUP,               // 上台阶: 收腿准备
    STEP_CHASSIS_APPROACH,    // 上台阶: 底盘向前微调贴紧台阶
    STEP_TOUCH_DOWN,          // 上台阶: 前腿找台阶, 后腿找地
    STEP_GLOBAL_LIFT,         // 上台阶: 全局顶升
    STEP_DRIVE_FWD,           // 上台阶: 轮子平移
    STEP_RETRACT,             // 上台阶: 收腿复位
    STEP_DONE,                // 上台阶完成

    STEP_DESCEND_FIND_EDGE,   // 下台阶: 寻崖模式
    STEP_DESCEND_SETUP,       // 下台阶: 收腿准备
    STEP_DESCEND_TOUCH,       // 下台阶: 触地
    STEP_DESCEND_GLOBAL_DOWN, // 下台阶: 全局下降
    STEP_DESCEND_DRIVE,       // 下台阶: 轮子平移
    STEP_DESCEND_RAISE,       // 下台阶: 抬起 最后是由上位机发指令让车复位

    STEP_WEAPON_HEAD_CLAMP,   // 夹武器头专属状态
    STEP_WEAPON_ROD_DOCK      // 对接武器杆专属状态
} ClimbingState_e;

// 轮子控制模式
typedef enum {
    WHEEL_MODE_ANGLE = 0,        // 正常斜坡角度跟踪
    WHEEL_MODE_CREEP,            // OMEGA模式抗台阶阻力
    WHEEL_MODE_CHASSIS_APPROACH,  // 调用底盘命令前进
    WHEEL_MODE_FIND_EDGE,         // 前腿寻崖 (前进)
    WHEEL_MODE_FIND_EDGE_BACKWARD // 后腿寻崖 (后退)
} WheelMode_e;

// 【核心架构】动作帧配置结构体
typedef struct {
    ClimbingState_e state_id;    // 状态标识
    float front_offset;          // 前腿目标偏置
    float rear_offset;           // 后腿目标偏置
    float wheel_travel_rad;      // 轮子角度增量
    uint32_t duration_ms;        // 本帧执行总时间
    uint32_t rear_delay_ms;      // 后腿延时时间(顶升用)
    uint8_t lift_pid_enable;     // 1: 开启重载PID，0: 恢复普通PID
    WheelMode_e wheel_mode;      // 轮子控制模式
    uint8_t is_descend_slope;    // 1: 轮子使用下台阶斜坡参数
} ActionFrame_t;

class ClimbingController {
private:
    // --- 表驱动执行引擎变量 ---
    const ActionFrame_t* current_seq_;  // 当前剧本(动作数组指针)
    uint8_t current_step_;              // 播放到第几步
    uint8_t total_steps_;               // 剧本总步数
    
    ClimbingState_e climb_state_;
    uint32_t state_tick_;
    uint8_t auto_running_;
    uint8_t is_zero_recorded_;
    uint8_t is_lift_pid_mode_;
    uint8_t rear_lift_delayed_flag_;

    // --- 机械参考锚点 ---
    float start_pos_front_;
    float start_pos_rear_;
    float wheel_target_angle_l_;
    float wheel_target_angle_r_;

    // --- 激光测距相关 ---
    // 前腿激光变量
    float laser_distance_;
    uint8_t laser_debounce_cnt_;
    // 后腿激光变量
    float laser_distance_rear_;
    uint8_t laser_debounce_cnt_rear_;

    // --- 执行器与规划器 ---
    QuinticPlanner planner_front_pos_{0.0f};
    QuinticPlanner planner_rear_pos_{0.0f};
    Class_Slope slope_wheel_l_angle_;
    Class_Slope slope_wheel_r_angle_;

    // --- 内部核心逻辑 ---
    void StartSequence(const ActionFrame_t* seq, uint8_t count, uint8_t start_index);
    void LoadNextFrame(void);
    void RecordSoftZero(void);
    void UpdateStateTargets(uint32_t current_time);
    void RunLiftAndWheelControl(void);
    void SetLiftPidMode(uint8_t enable_lift);
    void SetWheelSlopeStep(float wheel_step);

public:
    ClimbingController();

    void UpdateLaserDistance(float distance) { laser_distance_ = distance; }
    void UpdateLaserDistanceRear(float distance) { laser_distance_rear_ = distance; }
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

    ClimbingState_e GetState(void) const { return climb_state_; }
    uint8_t IsAutoRunning(void) const { return auto_running_; }

    uint8_t IsFindingEdgeFront(void) const;
    uint8_t IsFindingEdgeRear(void) const;

    void Init(FDCAN_HandleTypeDef *hcan);
    void TaskEntry1ms(void);
    void AutoTask1ms(void);
    void CAN_RxCallback(uint32_t std_id, uint8_t *data);

    // 执行器
    Class_Motor_DJI_C620 motor_lift_front_;
    Class_Motor_DJI_C620 motor_lift_rear_;
    Class_Motor_DJI_C620 motor_wheel_l_;
    Class_Motor_DJI_C620 motor_wheel_r_;

    // --- 对外公开接口 (保留完整功能) ---
    void AutoStart(void);
    void AutoStart20cm(void);
    void AutoStart40cm(void);
    void Prepare40cm(void);
    void AutoStartFromTouch20cm(void);
    void AutoStartFromTouch40cm(void);
    void DescendAutoStart(void);
    void DescendAutoStart20cm(void);
    void DescendAutoStart40cm(void);
    void InitPoseStart(void);
    void WeaponHeadClampStart(void); 
    void WeaponRodDockStart(void);   

    void DescendManualNext(void);
    void EmergencyStop(void);
};

#endif // TEST_FEEDBACK_CLIMBING_CONTROLLER_H
