#ifndef Laser_Task_H
#define Laser_Task_H

#include <stdint.h>

void LaserTask_Init(void);

void UART_Laser_Callback(uint8_t *Buffer, uint16_t Length);      // 前激光 huart10
void UART_Laser_Rear_Callback(uint8_t *Buffer, uint16_t Length); // 后激光 huart1


#endif // Laser_Task_H