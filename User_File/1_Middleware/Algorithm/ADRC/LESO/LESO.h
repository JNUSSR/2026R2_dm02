//
// Created by chengfeng on 2026/5/12.
//

#ifndef FOC_LESO_H
#define FOC_LESO_H

#ifndef PI
#define PI 3.14159265358979323846f
#endif

/**
 * @brief 一阶系统LESO观测器
 *
 */
class FirstOrderSystemESO {
public:
  FirstOrderSystemESO(float f, float ts, float b);
  float Get_b();
  float Get_x1();
  float Get_x2();
  void Init(float y);
  void Update(float y, float u);

private:
  bool InitFlag = false;
  float l1, l2;
  float Ts;
  float x1, x2;
  float b;
};

/**
 * @brief 二阶系统LESO观测器
 *
 */
class SecondOrderSystemESO {
public:
  SecondOrderSystemESO(float f, float ts, float b);
  float Get_b();
  float Get_x1();
  float Get_x2();
  float Get_x3();
  void Init(float y);
  void Update(float y, float u);

private:
  bool InitFlag = false;
  float l1, l2, l3;
  float Ts;
  float x1, x2, x3;
  float b;
};

#endif // FOC_LESO_H
