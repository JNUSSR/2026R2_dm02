//
// Created by chengfeng on 2026/4/18.
//

#ifndef TEST_FEEDBACK_ARM_H
#define TEST_FEEDBACK_ARM_H

#include "dvc_motor_dji.h"
#include "main.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

class BaseMotor {
public:
    virtual ~BaseMotor() = default;
    virtual void Set_Target_Angle(float angle) = 0; // 设置目标角度(弧度)
};

class MotorAdapter_C610 : public BaseMotor {
public:
    explicit MotorAdapter_C610(Class_Motor_DJI_C610& real_motor) : real_motor_(real_motor) {}
    void Set_Target_Angle(float angle) override {
        real_motor_.Set_Target_Angle(angle);
    }
private:
    Class_Motor_DJI_C610& real_motor_;
};

class MotorAdapter_C620 : public BaseMotor {
public:
    explicit MotorAdapter_C620(Class_Motor_DJI_C620& real_motor) : real_motor_(real_motor) {}
    void Set_Target_Angle(float angle) override {
        real_motor_.Set_Target_Angle(angle);
    }
private:
    Class_Motor_DJI_C620& real_motor_;
};

// ==========================================
// 1. 轨迹规划类 (5次多项式 S 型曲线)
// ==========================================
class QuinticPlanner {
public:
    enum State { UNPLANNING, PLANNING };

    explicit QuinticPlanner(float pos)
        : q0(pos), qf(pos), state(UNPLANNING), current_target(pos) {
    }

    // 规划一段新的轨迹
    void Plan(float start_pos, float target_pos, float time_duration);

    // 获取 dt 时间后的期望位置
    float GetNextPosition(float dt);

    bool IsPlanning() const;

    float GetCurrentTarget() const;

    void ForceSetPosition(float pos);

private:
    float q0, qf;
    float t, T;
    State state;
    float current_target;
};

class RobotJoint {
public:
    // 直接接收 BaseMotor，实现与具体硬件型号彻底解耦
    RobotJoint(BaseMotor &motor_ptr, float min_limit, float max_limit, float zero_pos, float dir, float ratio)
        : motor_(motor_ptr), v_min_(min_limit), v_max_(max_limit),
          pos_at_zero_(zero_pos), direction_(dir), transmission_ratio_(ratio) {}

    bool IsWithinLimits(float target_val) const;
    void Update(float current_val);

private:
    BaseMotor &motor_;
    float v_min_, v_max_;
    float pos_at_zero_;
    float direction_;
    float transmission_ratio_;
};

class PlannedJoint {
public:
    PlannedJoint(BaseMotor &motor, float min_limit, float max_limit, float zero_pos, float dir, float ratio, float dt)
        : joint_(motor, min_limit, max_limit, zero_pos, dir, ratio), planner_(zero_pos), dt_(dt) {}

    void Move(float target, float duration);
    bool IsMoving() const;
    void Update();

private:
    RobotJoint joint_;
    QuinticPlanner planner_;
    float dt_;
};

typedef void (*ActionCallback)(uint8_t arm_id);

struct ArmStep {
    float target_z, duration_z;
    float target_x, duration_x;
    float target_r, duration_r;

    ActionCallback custom_action;
};

class ArmSequencePlayer {
public:
    // 直接接收 PlannedJoint 引用
    ArmSequencePlayer(uint8_t arm_id,
                      PlannedJoint& joint_z,
                      PlannedJoint& joint_x,
                      PlannedJoint& joint_r)
        : arm_id_(arm_id), z_(joint_z), x_(joint_x), r_(joint_r), state_(IDLE), return_to_idle_when_done_(false), current_sequence_(nullptr) {}

    void Play(const ArmStep* sequence, uint16_t step_count, bool return_to_idle_when_done = false);
    void Stop();
    bool IsPlaying() const;
    void Update();

private:
    PlannedJoint& z_;
    PlannedJoint& x_;
    PlannedJoint& r_;

    enum State { IDLE, RUNNING_STEP, WAITING_STEP, DONE };
    State state_;

    uint8_t arm_id_;

    bool return_to_idle_when_done_;

    const ArmStep* current_sequence_;
    uint16_t total_steps_;
    uint16_t current_step_index_;
};

#endif // TEST_FEEDBACK_ARM_H
