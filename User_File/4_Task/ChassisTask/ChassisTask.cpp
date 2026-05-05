#include "ChassisTask.h"

#include "cmsis_os.h"
#include "uart_wheelchassis.h"


extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

static Class_Uart_Wheelchassis g_uart_wheelchassis;

// 全局目标速度，由上层调用接口设置
static volatile float g_target_vx = 0.0f;
static volatile float g_target_vy = 0.0f;
static volatile float g_target_vw = 0.0f;

/**
 * @brief CAN接收回调函数，将CAN报文分发到底盘控制模块
 * 
 * @param Header CAN接收的信息结构体
 * @param Buffer CAN接收的数据缓冲区
 */
void Chassis_CAN_Rx_Dispatch(FDCAN_HandleTypeDef* hfdcan, FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
	if (Buffer == nullptr)
	{
		return;
	}
	// if (hfdcan == nullptr || hfdcan->Instance != FDCAN2)
	// {
	// 	return;
	// }
	g_uart_wheelchassis.CAN_RxCallback(Header.Identifier, Buffer);
}

/**
 * @brief 设置底盘目标速度（供MAVLink等上层调用）
 *
 * @param vx 目标线速度X分量（m/s）
 * @param vy 目标线速度Y分量（m/s）
 * @param vw 目标角速度Z分量（rad/s）
 */
void Chassis_Set_Target(float vx, float vy, float vw)
{
	g_target_vx = vx;
	g_target_vy = vy;
	g_target_vw = vw;
}



/**
 * @brief 底盘任务初始化函数，主要进行底盘控制相关资源的初始化
 * 
 */
void Chassis_Task_Init(void)
{
	g_uart_wheelchassis.Init(&hfdcan2);
}

/**
 * @brief 立即停止底盘运动
 * 
 */
void Chassis_Task_StopImmediate(void)
{
    Chassis_Set_Target(0.0f, 0.0f, 0.0f);

    g_uart_wheelchassis.InitSlopePlanner();

    g_uart_wheelchassis.Set_Target_Velocity_X(0.0f);
    g_uart_wheelchassis.Set_Target_Velocity_Y(0.0f);
    g_uart_wheelchassis.Set_Target_Omega(0.0f);
}

/**
 * @brief 立即更新底盘控制状态
 */
void Chassis_Task_Update(void)
{
	g_uart_wheelchassis.WithSlopeTask(g_target_vx, g_target_vy, g_target_vw);

	// 调用底盘控制周期函数，更新PID（电机）
	for (int i = 0; i < 4; i++)
	{
		g_uart_wheelchassis.Chassis_Motor[i].TIM_Calculate_PeriodElapsedCallback();
	}
}

/**
 * @brief 底盘任务入口函数，1ms周期执行底盘控制逻辑
 * 
 */
void Chassis_Task(void* argument)
{
	(void)argument;
	Chassis_Task_Init();

	for (;;)
	{
        // 将全局目标速度传入底盘控制周期函数，进行斜坡规划，并将目标速度更新到底盘内的各个电机上
		g_uart_wheelchassis.WithSlopeTask(g_target_vx, g_target_vy, g_target_vw);

        // 调用底盘控制周期函数，更新PID（电机）
		for (int i = 0; i < 4; i++)
		{
			g_uart_wheelchassis.Chassis_Motor[i].TIM_Calculate_PeriodElapsedCallback();
		}
		
        // 通过CAN发送电机控制指令
		//TIM_CAN_PeriodElapsedCallback();

        // 1ms周期
		osDelay(1);
	}
}
