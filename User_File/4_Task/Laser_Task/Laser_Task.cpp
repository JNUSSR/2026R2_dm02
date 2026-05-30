#include "4_Task/Laser_Task/Laser_Task.h"
#include "3_Module/Laser/Laser.h"
#include "1_Middleware/Driver/UART/drv_uart.h"
#include "usart.h"
#include "climbingTask2.h"


Laser Laser_Module;

static void Laser_Rx_Push(const uint8_t *buffer, uint16_t length)
{
    static uint8_t acc[64] = {0};
    static size_t acc_len = 0;

    for (uint16_t i = 0; i < length; i++) {
        if (acc_len >= sizeof(acc)) {
            acc_len = 0;
        }

        acc[acc_len++] = buffer[i];

       if (acc_len >= 2 && acc[acc_len - 2] == 0x0D && acc[acc_len - 1] == 0x0A) {
            // 只有解析成功 (返回1) 时，才去给底盘送数据
            if (Laser_Module.Data_Process(acc, acc_len, &Laser_Module.Data) == 1) 
            {
                float distance = Laser_Module.Data.distance;
                uint8_t status = Laser_Module.Data.status;                 
                //实时喂给底盘控制器的寻崖系统
                Climbing_Get_Controller().UpdateLaserDistance(distance);
            }
            acc_len = 0;
        }
    }
}

void LaserTask_Init(void)
{
    UART_Init(&huart10, UART_Laser_Callback);
}

void UART_Laser_Callback(uint8_t *Buffer, uint16_t Length)
{
    Laser_Rx_Push(Buffer, Length);
}