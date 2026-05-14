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
#include "2_Device/BSP/BMI088/bsp_bmi088.h"
#include "2_Device/BSP/Power/bsp_power.h"
#include "4_Task/ChassisTask/ChassisTask.h"
#include "4_Task/ClampingTask/ClampingTask.h"
#include "4_Task/ClimbingTask/ClimbingTask2.h"
#include "4_Task/DrawKFSTask/DrawKFS_Task.h"
#include "1_Middleware/Driver/WDG/drv_wdg.h"
#include "1_Middleware/System/Timestamp/sys_timestamp.h"
#include "1_Middleware/Driver/Uart_printf/uart_printf.h"
#include "3_Module/arm/arm.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern ClampingController clampingCtrl;//初始化Task_Clamping使用
// 全局初始化完成标志位
bool init_finished = false;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief SPI2任务回调函数
 *
 */
void SPI2_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Tx_Length, uint16_t Rx_Length)
{
    (void) Tx_Buffer;
    (void) Rx_Buffer;
    (void) Tx_Length;
    (void) Rx_Length;

    if ((SPI2_Manage_Object.Activate_GPIOx == BMI088_ACCEL__SPI_CS_GPIO_Port && SPI2_Manage_Object.Activate_GPIO_Pin == BMI088_ACCEL__SPI_CS_Pin) || (SPI2_Manage_Object.Activate_GPIOx == BMI088_GYRO__SPI_CS_GPIO_Port && SPI2_Manage_Object.Activate_GPIO_Pin == BMI088_GYRO__SPI_CS_Pin))
    {
        BSP_BMI088.SPI_RxCpltCallback();
    }
}

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

extern Class_Motor_DJI_C620 Motor_Z;
extern Class_Motor_DJI_C610 Motor_X;
extern Class_Motor_DJI_C610 Motor_R;

static void CAN1_Global_Call_Back(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer) {
    switch (Header.Identifier) {
        case (0x202): {
            Climbing_CAN_Rx_Dispatch(Header, Buffer);
            break;
        }
        case (0x203): {
            Climbing_CAN_Rx_Dispatch(Header, Buffer);
            break;
        }
        case (0x204): {
            Climbing_CAN_Rx_Dispatch(Header, Buffer);
            break;
        }
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
            Motor_Z.CAN_RxCpltCallback();
            break;
        case (0x207):
            Clamping_CAN_Rx_Dispatch(Header, Buffer);
            break;
        case (0x206):
            Climbing_CAN_Rx_Dispatch(Header, Buffer);
            break;
    }
}




/**
 * @brief 每1ms调用一次
 *
 */
void Task1ms_Callback()
{
    // Class_Matrix_f32<3, 1> dbg_acc_o = BSP_BMI088.Get_Original_Accel();
    // Class_Matrix_f32<3, 1> dbg_gyro_o = BSP_BMI088.Get_Original_Gyro();
    // Class_Matrix_f32<3, 1> dbg_euler = BSP_BMI088.Get_Euler_Angle();
    // Class_Matrix_f32<3, 3> dbg_rot = BSP_BMI088.Get_Rotation_Matrix();
    // Class_Quaternion_f32 dbg_q = BSP_BMI088.Get_Quaternion();

    static int mod10 = 0;
    mod10++;
    if (mod10 == 10)
    {
        mod10 = 0;
        // uart_printf ("Accel: %f, %f, %f\r\n", dbg_acc_o[0][0], dbg_acc_o[1][0], dbg_acc_o[2][0]);
        // uart_printf ("Gyro: %f, %f, %f\r\n", dbg_gyro_o[0][0], dbg_gyro_o[1][0], dbg_gyro_o[2][0]);
        // uart_printf ("Euler: %f, %f, %f\r\n", dbg_euler[0][0], dbg_euler[1][0], dbg_euler[2][0]);
    }

    // 已经在DJI_M3508_Task的TIM_1ms_CAN_PeriodElapsedCallback中调用了TIM_1ms_CAN_PeriodElapsedCallback, 这里就不再调用了
    // 在1ms定时器中断中调用CAN回调函数, 防止任务多次重复发送导致电机疯转
    TIM_1ms_CAN_PeriodElapsedCallback();
    
    // 喂狗
    TIM_1ms_IWDG_PeriodElapsedCallback();

    

    // static int mod128 = 0;
    // mod128++;
    // if (mod128 == 128)
    // {
    //     mod128 = 0;

    //     BSP_BMI088.TIM_128ms_Calculate_PeriodElapsedCallback();
    // }
}

/**
 * @brief 每125us调用一次
 *
 */
void Task125us_Callback()
{
    BSP_BMI088.TIM_125us_Calculate_PeriodElapsedCallback();
}

/**
 * @brief 每10us调用一次
 *
 */
void Task10us_Callback()
{
    BSP_BMI088.TIM_10us_Calculate_PeriodElapsedCallback();
}

/**
 * @brief 初始化任务
 *
 */
void Task_Init()
{
    SYS_Timestamp.Init(&htim5);

    // 陀螺仪的SPI
    SPI_Init(&hspi2, SPI2_Callback);

    //串口回调测试
    // UART_Init(&huart7, &USART_DEBUG_CALLBACK);

    // 电机的CAN
    // CAN_Init(&hfdcan1, CAN1_Callback);
    CAN_Init(&hfdcan1, CAN1_Global_Call_Back);
    CAN_Init(&hfdcan2, CAN2_Global_Call_Back);

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


    //再Task_Init中初始化Task_Clamping，解决了按复位按钮后电机无法正常运行的问题
    clampingCtrl.Init(&hfdcan2);

    // BSP_BMI088.Init();

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

    if (GPIO_Pin == BMI088_ACCEL__INTERRUPT_Pin || GPIO_Pin == BMI088_GYRO__INTERRUPT_Pin)
    {
        BSP_BMI088.EXTI_Flag_Callback(GPIO_Pin);
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
