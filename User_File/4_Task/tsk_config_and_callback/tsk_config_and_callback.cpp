/**
 * @file tsk_config_and_callback.cpp
 * @author 
 * @brief 临时任务调度测试用函数, 后续用来存放个人定义的回调函数以及若干任务
 *        Task_Init在freertos.c的MX_FREERTOS_Init中被调用, 用于初始化回调函数等
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"

#include "fdcan.h"
#include "spi.h"
#include "adc.h"
#include "1_Middleware/Driver/CAN/drv_can.h"
#include "1_Middleware/Driver/ADC/drv_adc.h"
#include "1_Middleware/Driver/SPI/drv_spi.h"
#include "1_Middleware/Driver/UART/drv_uart.h"
#include "2_Device/BSP/Power/bsp_power.h"
#include "4_Task/ChassisTask/ChassisTask.h"
#include "4_Task/ClampingTask/ClampingTask.h"
#include "4_Task/ClimbingTask/ClimbingTask2.h"
#include "4_Task/DrawKFSTask/DrawKFS_Task.h"
#include "1_Middleware/Driver/WDG/drv_wdg.h"
#include "1_Middleware/System/Timestamp/sys_timestamp.h"
#include "1_Middleware/Driver/Uart_printf/uart_printf.h"
#include "3_Module/arm/arm.h"
#include "3_Module/Climbing/climbing_controller.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern ClampingController clampingCtrl;//初始化Task_Clamping使用
// 全局初始化完成标志位
bool init_finished = false;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

void USART_DEBUG_CALLBACK(uint8_t *Buffer, uint16_t Length)
{
    if(Buffer[Length - 1] == '\n')
    {
        uart_printf("echo: %s",Buffer);
    }
}

/**
 * @brief 每3600s调用一次
 *
 */
void Task3600s_Callback()
{
    SYS_Timestamp.TIM_3600s_PeriodElapsedCallback();
}

/**
 * @brief 每1s调用一次
 *
 */
void Task1s_Callback()
{
}

// extern "C" void Chassis_CAN_Rx_Dispatch(FDCAN_HandleTypeDef *hcan, Struct_CAN_Rx_Buffer *Rx_Buffer);

//Climbing
extern ClimbingController climbingCtrl;

// ARM电机
extern Class_Motor_DJI_C620 Motor_Z;
extern Class_Motor_DJI_C610 Motor_X;
extern Class_Motor_DJI_C620 Motor_R;

extern Class_Motor_DJI_C620 Motor_Z_R;
extern Class_Motor_DJI_C610 Motor_X_R;
extern Class_Motor_DJI_C620 Motor_R_R;
 
static void CAN1_Global_Call_Back(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer) {
    switch (Header.Identifier) {
        case (0x201): {
            climbingCtrl.motor_lift_front_.CAN_RxCpltCallback();
            break;
        }
        case (0x202):
            Clamping_CAN_Rx_Dispatch(Header, Buffer);
            break;
        case (0x205):
            Motor_Z.CAN_RxCpltCallback();
            break;
        case (0x206):
            Motor_X.CAN_RxCpltCallback();
            break;
        case (0x207):
            Motor_R.CAN_RxCpltCallback();
            break;
        default:
            break;
    }
}

void CAN2_Global_Call_Back(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer) {
    switch (Header.Identifier) {
        case (0x201):
        case (0x202):
        case (0x203):
        case (0x204):
            Chassis_CAN_Rx_Dispatch(&hfdcan2, Header, Buffer);
            break;
        case (0x205):
            climbingCtrl.motor_wheel_l_.CAN_RxCpltCallback();
            break;
        case (0x206):
            climbingCtrl.motor_wheel_r_.CAN_RxCpltCallback();
            break;
    }
}

void CAN3_Global_Call_Back(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer) {
    switch (Header.Identifier) {
        case (0x201):
            Motor_Z_R.CAN_RxCpltCallback();
            break;
        case (0x202):
            Motor_X_R.CAN_RxCpltCallback();
            break;
        case (0x203):
            Motor_R_R.CAN_RxCpltCallback();
            break;
        case (0x204):
            climbingCtrl.motor_lift_rear_.CAN_RxCpltCallback();
            break;
        default:
            break;
    }
}


/**
 * @brief 每1ms调用一次
 *
 */
void Task1ms_Callback()
{
    // 在1ms定时器中断中调用CAN回调函数, 防止任务多次重复发送导致电机疯转
    TIM_1ms_CAN_PeriodElapsedCallback();
    
    // 喂狗
    TIM_1ms_IWDG_PeriodElapsedCallback();
}

/**
 * @brief 每125us调用一次
 *
 */
void Task125us_Callback()
{
    
}

/**
 * @brief 每10us调用一次
 *
 */
void Task10us_Callback()
{
    
}

/**
 * @brief 初始化任务
 *
 */
void Task_Init()
{
    SYS_Timestamp.Init(&htim5);

    //串口回调测试
    // UART_Init(&huart7, &USART_DEBUG_CALLBACK);

    // 电机的CAN
    // CAN_Init(&hfdcan1, CAN1_Callback);
    CAN_Init(&hfdcan1, CAN1_Global_Call_Back);
    CAN_Init(&hfdcan2, CAN2_Global_Call_Back);
    CAN_Init(&hfdcan3, CAN3_Global_Call_Back); 

    // 电源相关ADC
    ADC_Init(&hadc1, 1);

    // 打开DC5输出后再初始化BMI088
    BSP_Power.Init(false, false, true);

    // 定时器中断初始化
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);
    HAL_TIM_Base_Start_IT(&htim8);


    //在Task_Init中初始化Task_Clamping，解决了按复位按钮后电机无法正常运行的问题
    clampingCtrl.Init(&hfdcan1);

    // 标记初始化完成
    init_finished = true;
}

/**
 * @brief 前台循环任务
 *
 */
void Task_Loop()
{
}

/**
 * @brief GPIO中断回调函数
 *
 * @param GPIO_Pin 中断引脚
 */
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (!init_finished)
    {
        return;
    }
}

/**
 * @brief 定时器中断回调函数
 *
 * @param htim
 */
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (!init_finished)
    {
        return;
    }

    // 选择回调函数
    if (htim->Instance == TIM4)
    {
        Task10us_Callback();
    }
    else if (htim->Instance == TIM5)
    {
        Task3600s_Callback();
    }
    else if (htim->Instance == TIM6)
    {
        Task1s_Callback();
    }
    else if (htim->Instance == TIM7)
    {
        Task1ms_Callback();
    }
    else if (htim->Instance == TIM8)
    {
        Task125us_Callback();
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
