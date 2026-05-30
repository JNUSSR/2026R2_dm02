#ifndef Laser_Task_H
#define Laser_Task_H

#include <stdint.h>

void LaserTask_Init(void);

void UART_Laser_Callback(uint8_t *Buffer, uint16_t Length);



#endif // Laser_Task_H