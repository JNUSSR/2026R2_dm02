#ifndef __CYLINDER_H
#define __CYLINDER_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

// GPIO定义(气缸控制引脚)
#define Cylinder_CYLINDER_GPIO_PORT1  GPIOE
#define Cylinder_CYLINDER_GPIO_PIN1   GPIO_PIN_9
#define Cylinder_CYLINDER_GPIO_PORT2  GPIOE
#define Cylinder_CYLINDER_GPIO_PIN2   GPIO_PIN_4
#define Cylinder_CYLINDER_POWER_GPIO_PORT GPIOF//引脚供电控制，默认为常电，拉高供电
#define Cylinder_CYLINDER_POWER_GPIO_PIN GPIO_PIN_1


void Cylinder_Init();
void Cylinder1_Push();
void Cylinder2_Push();
void Cylinder_Push();
void Cylinder_Pull();
void Cylinder1_Pull();
void Cylinder2_Pull();


#ifdef __cplusplus
}
#endif

    
#endif