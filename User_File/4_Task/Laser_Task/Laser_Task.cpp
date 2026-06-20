#include "4_Task/Laser_Task/Laser_Task.h"
#include "3_Module/Laser/Laser.h"
#include "1_Middleware/Driver/UART/drv_uart.h"
#include "usart.h"
#include "climbingTask2.h"


Laser Laser_Module;
Laser Laser_Module_Front;
Laser Laser_Module_Rear;

// ---------------- 前腿激光 (UART1) ----------------
static void Laser_Rx_Push_Front(const uint8_t *buffer, uint16_t length)
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
            if (Laser_Module_Front.Data_Process(acc, acc_len, &Laser_Module_Front.Data) == 1) 
            {             
                //实时喂给底盘控制器的寻崖系统
                Climbing_Get_Controller().UpdateLaserDistance(Laser_Module_Front.Data.distance);
            }
            acc_len = 0;
        }
    }
}
void UART_Laser_Callback(uint8_t *Buffer, uint16_t Length) {
    Laser_Rx_Push_Front(Buffer, Length);
}

// ---------------- 后腿激光 (UART10) ----------------
static void Laser_Rx_Push_Rear(const uint8_t *buffer, uint16_t length)
{
    // ⚠️ 必须用独立的 static 变量，防止和前激光中断打架
    static uint8_t acc_rear[64] = {0};
    static size_t acc_len_rear = 0;
    for (uint16_t i = 0; i < length; i++) {
        if (acc_len_rear >= sizeof(acc_rear)) acc_len_rear = 0;
        acc_rear[acc_len_rear++] = buffer[i];

       if (acc_len_rear >= 2 && acc_rear[acc_len_rear - 2] == 0x0D && acc_rear[acc_len_rear - 1] == 0x0A) {
            if (Laser_Module_Rear.Data_Process(acc_rear, acc_len_rear, &Laser_Module_Rear.Data) == 1) {
                // 将解析出来的后方距离喂给控制器专属接口
                Climbing_Get_Controller().UpdateLaserDistanceRear(Laser_Module_Rear.Data.distance);
            }
            acc_len_rear = 0;
        }
    }
}

void UART_Laser_Rear_Callback(uint8_t *Buffer, uint16_t Length) {
    Laser_Rx_Push_Rear(Buffer, Length);
}

void LaserTask_Init(void)
{
    UART_Init(&huart1, UART_Laser_Callback);       // 绑定前腿激光
    UART_Init(&huart10, UART_Laser_Rear_Callback);   // 绑定后腿激光
}
