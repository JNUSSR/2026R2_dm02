//
// Created by chengfeng on 2026/5/3.
//

#include "Uart_Task.h"
#include "mavlink.h"
#include "cmsis_os2.h"
#include "drv_uart.h"
#include "main.h"
#include "ChassisTask.h"
#include "climbingTask2.h"
#include "ClampingTask.h"


uint16_t rx_len = 0;

// 左臂标志位
extern bool drawkfs_flag;
extern bool idle_flag;
extern bool drawkfs_40cm_flag;
extern bool drawkfs_below20cm_flag;
extern bool putkfs_ordertwo_flag;
extern bool putkfs_ordertwo_prepare_flag;
extern bool putkfs_ordertwo_release_flag;
extern bool putkfs_orderthree_flag;

// 右臂标志位
extern bool right_drawkfs_flag;
extern bool right_idle_flag;
extern bool right_drawkfs_40cm_flag;
extern bool right_drawkfs_below20cm_flag;
extern bool right_putkfs_ordertwo_flag;
extern bool right_putkfs_ordertwo_prepare_flag;
extern bool right_putkfs_ordertwo_release_flag;
extern bool right_putkfs_orderthree_flag;

extern osSemaphoreId_t uartSemaphoreHandle;

void UartCallback(uint8_t *Buffer, uint16_t Length);

void UartTask() {
    UART_Init(&huart7, &UartCallback);
    for (;;) {
        osSemaphoreAcquire(uartSemaphoreHandle, osWaitForever);
        if (rx_len > 0) {
            mavlink_status_t status;
            mavlink_message_t msg;
            int chan = 0;
            // 将接收到的整块数据，逐字节喂给 MAVLink 解析器

            for (uint16_t i = 0; i < rx_len; i++) {
                // mavlink_parse_char 返回 1 表示成功解析出一帧完整的消息
                if (mavlink_parse_char(chan, UART7_Manage_Object.Rx_Buffer_Ready[i], &msg, &status)) {
                    if (msg.msgid == MAVLINK_MSG_ID_ARM_CONTROL) {
                        mavlink_arm_control_t arm_control;
                        mavlink_msg_arm_control_decode(&msg, &arm_control);

                        uint8_t target = arm_control.target_arm;
                        uint8_t cmd    = arm_control.action_cmd;

                        // 1. 如果目标是 左臂(0) 或 双臂(2)，则触发左臂对应的标志位
                        if (target == ARM_SELECT_LEFT || target == ARM_SELECT_BOTH) {
                            switch (cmd) {
                                case ARM_ACTION_IDLE:             idle_flag = true; break;
                                case ARM_DRAW_KFS_20cm:           drawkfs_flag = true; break;
                                case ARM_DRAW_KFS_40cm:           drawkfs_40cm_flag = true; break;
                                case ARM_DRAW_KFS_BELOW_20cm:     drawkfs_below20cm_flag = true; break;
                                case ARM_PUT_KFS_ORDER_TWO:       putkfs_ordertwo_flag = true; break;
                                case ARM_PUT_KFS_ORDER_TWO_PREPARE: putkfs_ordertwo_prepare_flag = true; break;
                                case ARM_PUT_KFS_ORDER_TWO_RELEASE: putkfs_ordertwo_release_flag = true; break;
                                case ARM_PUT_KFS_ORDER_THREE: putkfs_orderthree_flag = true; break;
                            }
                        }

                        // 2. 如果目标是 右臂(1) 或 双臂(2)，则触发右臂对应的标志位
                        if (target == ARM_SELECT_RIGHT || target == ARM_SELECT_BOTH) {
                            switch (cmd) {
                                case ARM_ACTION_IDLE:             right_idle_flag = true; break;
                                case ARM_DRAW_KFS_20cm:           right_drawkfs_flag = true; break;
                                case ARM_DRAW_KFS_40cm:           right_drawkfs_40cm_flag = true; break;
                                case ARM_DRAW_KFS_BELOW_20cm:     right_drawkfs_below20cm_flag = true; break;
                                case ARM_PUT_KFS_ORDER_TWO:       right_putkfs_ordertwo_flag = true; break;
                                case ARM_PUT_KFS_ORDER_TWO_PREPARE: right_putkfs_ordertwo_prepare_flag = true; break;
                                case ARM_PUT_KFS_ORDER_TWO_RELEASE: right_putkfs_ordertwo_release_flag = true; break;
                                case ARM_PUT_KFS_ORDER_THREE: right_putkfs_orderthree_flag = true; break;
                            }
                        }
                    } else if (msg.msgid == MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD) {
                        mavlink_chassis_velocity_cmd_t chassis_velocity_cmd;
                        mavlink_msg_chassis_velocity_cmd_decode(&msg, &chassis_velocity_cmd);
                        Chassis_Set_Target(chassis_velocity_cmd.vx,
                                           chassis_velocity_cmd.vy,
                                           chassis_velocity_cmd.vw);
                    } else if (msg.msgid == MAVLINK_MSG_ID_CLIMBING_CMD) {
                        mavlink_climbing_cmd_t climbing_cmd;
                        mavlink_msg_climbing_cmd_decode(&msg, &climbing_cmd);

                        switch (climbing_cmd.command) {
                            case CLIMBING_CMD_INIT_POSE:
                                Climbing_Init_Pose_Start();
                                break;
                            case CLIMBING_CMD_PREPARE_40CM:
                                Climbing_Prepare_40cm();
                                break;
                            case CLIMBING_CMD_EXECUTE_UP_20CM:
                                Climbing_Auto_Start_From_Touch_20cm();
                                break;
                            case CLIMBING_CMD_EXECUTE_UP_40CM:
                                Climbing_Auto_Start_From_Touch_40cm();
                                break;
                            case CLIMBING_CMD_EXECUTE_DOWN_20CM:
                                Climbing_Descend_Auto_Start_20cm();
                                break;
                            case CLIMBING_CMD_WEAPON_HEAD_CLAMP_START:
                                Climbing_WeaponHeadClampStart();
                                break;
                            case CLIMBING_CMD_WEAPON_ROD_DOCK_START:
                                Climbing_WeaponRodDockStart();
                                break;
                            case CLIMBING_CMD_EXECUTE_DOWN_40CM:
                                Climbing_Descend_Auto_Start_40cm();
                                break;
                        }
                    } else if (msg.msgid == MAVLINK_MSG_ID_CLAMPING_CMD) {
                        mavlink_clamping_cmd_t clamping_cmd;
                        mavlink_msg_clamping_cmd_decode(&msg, &clamping_cmd);

                        switch (clamping_cmd.command) {
                            case CLAMPING_CMD_MOVE_TO_PARALLEL:
                                Clamping_Get_Controller().MoveToClampAngle();
                                break;
                            case CLAMPING_CMD_GRAB:
                                Clamping_Get_Controller().OpenSolenoid();
                                break;
                            case CLAMPING_CMD_MOVE_TO_RESET:
                                Clamping_Get_Controller().MoveToResetAngle();
                                break;
                            case CLAMPING_CMD_RELEASE:
                                Clamping_Get_Controller().ReleaseSolenoid();
                                break;
                            case CLAMPING_CMD_MOVE_TO_DOCK:
                                Clamping_Get_Controller().MoveToDockAngle();
                                break;
                            case CLAMPING_CMD_ADJUST:
                                Clamping_Auto_Adjust();
                                break;
                            case CLAMPING_CMD_START:
                                Clamping_Auto_Start();
                                break;
                            case CLAMPING_CMD_MOVE_TO_AIRTAG:
                                Clamping_Get_Controller().MoveToAirtag();
                                break;
                        }
                    }
                }
            }
            rx_len = 0;
            // 重新开启 DMA + 空闲中断接收，准备接下一批数据
            // 注意：drv_uart.c中开启dma空闲中断的函数被注释
            HAL_UARTEx_ReceiveToIdle_DMA(&huart7, UART7_Manage_Object.Rx_Buffer_Active, UART_BUFFER_SIZE);
        }
    }
}

void UartCallback(uint8_t *Buffer, uint16_t Length) {
    rx_len = Length;
    osSemaphoreRelease(uartSemaphoreHandle);
}
