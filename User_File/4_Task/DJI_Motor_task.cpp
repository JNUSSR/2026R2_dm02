#include "4_Task/DJI_Motor_task.h"

#include "cmsis_os.h"
#include "fdcan.h"
#include "1_Middleware/Driver/CAN/drv_can.h"
#include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"

static Class_Motor_DJI_C620 dji_m3508_motor;

/**
 * @brief 初始化M3508电机对象及其PID
 */
extern "C" void DJI_M3508_Task_Init(void)
{
    dji_m3508_motor.PID_Omega.Init(1.0f, 1.0f, 0.0f, 0.0f, 2.0f, 4.0f);
    dji_m3508_motor.Init(&hfdcan1, Motor_DJI_ID_0x201, Motor_DJI_Control_Method_OMEGA);
}

/**
 * @brief CAN接收完成回调的包装函数
 * @param Rx_Data 指向接收到的CAN数据缓冲区的指针
 */
extern "C" void DJI_M3508_CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    dji_m3508_motor.CAN_RxCpltCallback();
}

/**
 * @brief M3508电机的FreeRTOS任务入口函数
 * @param argument 任务参数 (未使用)
 */
extern "C" void DJI_M3508_Task(void *argument)
{
    DJI_M3508_Task_Init();
    (void) argument;
    uint32_t time_counter = 0;
    for (;;)
    {
        dji_m3508_motor.Set_Target_Omega(5.0f);

        if ((time_counter % 100U) == 0U)
        {
            dji_m3508_motor.TIM_100ms_Alive_PeriodElapsedCallback();
        }

        dji_m3508_motor.TIM_Calculate_PeriodElapsedCallback();
        TIM_1ms_CAN_PeriodElapsedCallback();

        time_counter++;
        osDelay(1);
    }
}
