//
// Created by chengfeng on 2026/5/12.
//

#include "LESO.h"

/**
 * @brief 构造一阶LESO
 *
 * @param f 观测器带宽
 * @param ts 采样周期
 * @param b 输入增益
 */
FirstOrderSystemESO::FirstOrderSystemESO(float f, float ts, float b)
    : Ts(ts), x1(0.0f), x2(0.0f), b(b) {
  float omega = 2 * PI * f;
  l1 = 2 * omega;
  l2 = omega * omega;
};

/**
 * @brief 更新一阶LESO状态
 *
 * @param y 当前系统输出
 * @param u 当前控制输入
 */
void FirstOrderSystemESO::Update(float y, float u) {
  float e = y - x1;

  float x1_next = x1 + (x2 + l1 * e + b * u) * Ts;
  float x2_next = x2 + l2 * e * Ts;

  x1 = x1_next;
  x2 = x2_next;
}

/**
 * @brief 初始化一阶LESO状态
 *
 * @param y 当前系统输出
 */
void FirstOrderSystemESO::Init(float y) {
  if (!InitFlag) {
    x1 = y;
    x2 = 0.0f;
    InitFlag = true;
  }
}

/**
 * @brief 获取一阶LESO输入增益b
 *
 * @return float 输入增益b
 */
float FirstOrderSystemESO::Get_b() { return b; }

/**
 * @brief 获取一阶LESO状态x1
 *
 * @return float 观测状态x1
 */
float FirstOrderSystemESO::Get_x1() { return x1; }

/**
 * @brief 获取一阶LESO状态x2
 *
 * @return float 扰动观测量x2
 */
float FirstOrderSystemESO::Get_x2() { return x2; }

/**
 * @brief 构造二阶LESO
 *
 * @param f 观测器带宽
 * @param ts 采样周期
 * @param b 输入增益
 */
SecondOrderSystemESO::SecondOrderSystemESO(float f, float ts, float b)
: Ts(ts), x1(0.0f), x2(0.0f), x3(0.0f), b(b) {
  float omega = 2 * PI * f;
  l1 = 3 * omega;
  l2 = 3 * omega * omega;
  l3 = omega * omega * omega;
}

/**
 * @brief 初始化二阶LESO状态
 *
 * @param y 当前系统输出
 */
void SecondOrderSystemESO::Init(float y) {
  if (!InitFlag) {
    x1 = y;
    x2 = 0.0f;
    x3 = 0.0f;
    InitFlag = true;
  }
}

/**
 * @brief 更新二阶LESO状态
 *
 * @param y 当前系统输出
 * @param u 当前控制输入
 */
void SecondOrderSystemESO::Update(float y, float u) {
  float e = y - x1;
  x1 = x1 + (l1 * e + x2) * Ts;
  x2 = x2 + (l2 * e + x3 + b * u) * Ts;
  x3 = x3 + l3 * e * Ts;
}

/**
 * @brief 获取二阶LESO输入增益b
 *
 * @return float 输入增益b
 */
float SecondOrderSystemESO::Get_b() {
  return b;
}

/**
 * @brief 获取二阶LESO状态x1
 *
 * @return float 观测状态x1
 */
float SecondOrderSystemESO::Get_x1() {
  return x1;
}

/**
 * @brief 获取二阶LESO状态x2
 *
 * @return float 观测状态x2
 */
float SecondOrderSystemESO::Get_x2() {
  return x2;
}

/**
 * @brief 获取二阶LESO状态x3
 *
 * @return float 扰动观测量x3
 */
float SecondOrderSystemESO::Get_x3() {
  return x3;
}
