//
// Created by chengfeng on 2026/5/13.
//

#ifndef TEST_FEEDBACK_ADRC_H
#define TEST_FEEDBACK_ADRC_H

#include "LESO.h"

class FirstOrderSystemADRC {
public:
  // eso_f : 观测器带宽(Hz)  eso_dt : 采样时间   eso_b : 和模型相关，可从大往小试调
  // kp : pid控制器的Kp参数 max_out: 输出限幅
  FirstOrderSystemADRC(float eso_f, float dt, float eso_b, float pid_f, float max_out)
      : LESO(eso_f, dt, eso_b), MaxOut(max_out), Ts(dt) {
    float omega = 2 * PI * pid_f;
    Kp = omega;
  }

  float Update(float target, float feedback) {
    float error = target - feedback;
    float pid_output = Kp * error;
    float output = (pid_output - LESO.Get_x2()) / LESO.Get_b();
    float output_limit = ClampAbs(output, MaxOut);
    LESO.Update(feedback,output_limit);
    return output_limit;
  }

  void Init(float feedback) {
    LESO.Init(feedback);
  }

  void Set_Kp(float kp) {
    Kp = kp;
  }

  void Set_MaxOut(float max_out) {
    MaxOut = max_out;
  }

  float Get_x1() { return LESO.Get_x1(); }
  float Get_x2() { return LESO.Get_x2(); }
  float Get_b() { return LESO.Get_b(); }

private:
  FirstOrderSystemESO LESO;
  float Kp;
  float MaxOut;
  float Ts;

  float ClampAbs(float value, float limit) {
    if (value > limit) {
      return limit;
    }
    if (value < -limit) {
      return -limit;
    }
    return value;
  }
};

class SecondOrderSystemADRC {
public:
  SecondOrderSystemADRC(float eso_f, float eso_dt, float eso_b, float pid_f, float max_out)
      : LESO(eso_f, eso_dt, eso_b), MaxOut(max_out) {
    float omega = 2 * PI * pid_f;
    Kp = omega * omega;
    Kd = 2 * omega;
  }

  //当target_vel = 0.0f时，为位置环，速度那一项是阻尼项
  //当已知target_vel时，可代入，相当于带前馈的位置环，轨迹规划能同时给出位置和速度，用上效果应该会更好

  //使用外部测量的vel值
  float Update(float target_pos, float pos_feedback, float vel_feedback, float target_vel) {
    float pid_output = Kp * (target_pos - pos_feedback) + Kd * (target_vel - vel_feedback);
    float output = (pid_output - LESO.Get_x3()) / LESO.Get_b();
    float output_limit = ClampAbs(output, MaxOut);
    LESO.Update(pos_feedback, output_limit);
    return output_limit;
  }

  //使用LESO观测的vel值
  float Update(float target_pos, float pos_feedback, float target_vel) {
    float pid_output = Kp * (target_pos - pos_feedback) + Kd * (target_vel - LESO.Get_x2());
    float output = (pid_output - LESO.Get_x3()) / LESO.Get_b();
    float output_limit = ClampAbs(output, MaxOut);
    LESO.Update(pos_feedback, output_limit);
    return output_limit;
  }

  void Init(float feedback) {
    LESO.Init(feedback);
  }

  void Set_Kp(float kp) {
    Kp = kp;
  }

  void Set_Kd(float kd) {
    Kd = kd;
  }

  void Set_MaxOut(float max_out) {
    MaxOut = max_out;
  }

  float Get_x1() { return LESO.Get_x1(); }
  float Get_x2() { return LESO.Get_x2(); }
  float Get_x3() { return LESO.Get_x3(); }
  float Get_b() { return LESO.Get_b(); }

private:
  SecondOrderSystemESO LESO;
  float Kp,Kd;
  float MaxOut;
  float ClampAbs(float value, float limit) {
    if (value > limit) {
      return limit;
    }
    if (value < -limit) {
      return -limit;
    }
    return value;
  }
};

#endif // TEST_FEEDBACK_ADRC_H
