//
// Created by chengfeng on 2026/4/18.
//

#include "arm.h"
#include "drv_can.h"


// // 物理传动参数宏定义 (半径单位：米)
// #define M3508_R 0.01526f
// #define M2006_R 0.01450f
// // 旋转轴的减速比 (电机转动弧度 / 底盘实际转动弧度)
// #define YAW_RATIO 3.0f

void QuinticPlanner::Plan(float start_pos, float target_pos, float time_duration) {
    if (time_duration == 0) {
        return;
    }
    q0 = start_pos;
    qf = target_pos;
    T = time_duration;
    t = 0.0f;
    state = PLANNING;
    current_target = start_pos;
}

float QuinticPlanner::GetNextPosition(float dt) {
    if (state == PLANNING) {
        t += dt;
        if (t >= T) {
            t = T;
            state = UNPLANNING;
            current_target = qf;
        } else {
            // 归一化时间
            float tau = t / T;
            float tau3 = tau * tau * tau;

            // 5次多项式 S型平滑曲线 (保证v0=0, a0=0, vf=0, af=0)
            // 公式提取公因式优化算力：tau^3 * (10 - 15*tau + 6*tau^2)
            float scale = tau3 * (10.0f - 15.0f * tau + 6.0f * tau * tau);
            current_target = q0 + (qf - q0) * scale;
        }
    }
    return current_target;
}

bool QuinticPlanner::IsPlanning() const {
    return state == PLANNING;
}

float QuinticPlanner::GetCurrentTarget() const {
    return current_target;
}

void QuinticPlanner::ForceSetPosition(float pos) {
    q0 = pos;
    qf = pos;
    current_target = pos;
    state = UNPLANNING; // 明确设定为空闲状态
}

bool RobotJoint::IsWithinLimits(float target_val) const {
    return (target_val >= v_min_ && target_val <= v_max_);
}

void RobotJoint::Update(float current_val) {
    if (IsWithinLimits(current_val)) {
        float target_angle = (current_val - pos_at_zero_) * direction_ * transmission_ratio_;
        motor_.Set_Target_Angle(target_angle); // 触发多态，自动调用适配器
    }
}

void PlannedJoint::Move(float target, float duration) {
    // 只有在空闲且目标合法的情况下才启动新规划
    if (!planner_.IsPlanning() && joint_.IsWithinLimits(target)) {
        planner_.Plan(planner_.GetCurrentTarget(), target, duration);
    }
}

bool PlannedJoint::IsMoving() const {
    return planner_.IsPlanning();
}

void PlannedJoint::Update() {
    joint_.Update(planner_.GetNextPosition(dt_));
}

void ArmSequencePlayer::Play(const ArmStep* sequence, uint16_t step_count, bool return_to_idle_when_done) {
    if (sequence == nullptr || step_count == 0) return;
    current_sequence_ = sequence;
    total_steps_ = step_count;
    current_step_index_ = 0;
    return_to_idle_when_done_ = return_to_idle_when_done;
    state_ = RUNNING_STEP;
}

void ArmSequencePlayer::Stop() {
    state_ = IDLE;
}

bool ArmSequencePlayer::IsPlaying() const {
    return state_ == RUNNING_STEP || state_ == WAITING_STEP;
}

void ArmSequencePlayer::Update() {
    if (state_ == IDLE) return;

    if (state_ == RUNNING_STEP) {
        const ArmStep& step = current_sequence_[current_step_index_];

        // 优先执行自定义动作 (例如 GPIO 控制、printf 打印等)
        if (step.custom_action != nullptr) {
            step.custom_action(arm_id_); 
        }

        // 如果 duration > 0 才下发移动指令，否则保持原位不动
        if (step.duration_z > 0.0f) z_.Move(step.target_z, step.duration_z);
        if (step.duration_x > 0.0f) x_.Move(step.target_x, step.duration_x);
        if (step.duration_r > 0.0f) r_.Move(step.target_r, step.duration_r);

        state_ = WAITING_STEP;
    }
    else if (state_ == WAITING_STEP) {
        // 检查这一步是否所有的轴都已经运动完毕
        if (!z_.IsMoving() && !x_.IsMoving() && !r_.IsMoving()) {
            current_step_index_++;
            if (current_step_index_ >= total_steps_) {
                state_ = return_to_idle_when_done_ ? IDLE : DONE; // 整个动作序列完成
            } else {
                state_ = RUNNING_STEP; // 继续下一步
            }
        }
    }
}
