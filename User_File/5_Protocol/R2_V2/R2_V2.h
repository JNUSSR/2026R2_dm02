/** @file
 *  @brief MAVLink comm protocol generated from R2_V2.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_R2_V2_H
#define MAVLINK_R2_V2_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_R2_V2.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#define MAVLINK_R2_V2_XML_HASH -53279337908117539

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{200, 230, 1, 1, 0, 0, 0}, {201, 211, 12, 12, 0, 0, 0}, {202, 132, 1, 1, 0, 0, 0}, {203, 109, 1, 1, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_R2_V2

// ENUM DEFINITIONS


/** @brief Arm action enum */
#ifndef HAVE_ENUM_ARM_ACTION_CMD
#define HAVE_ENUM_ARM_ACTION_CMD
typedef enum ARM_ACTION_CMD
{
   ARM_ACTION_IDLE=0, /* IDLE/STOP current action | */
   ARM_DRAW_KFS=1, /* Draw KFS | */
   ARM_ACTION_CMD_ENUM_END=2, /*  | */
} ARM_ACTION_CMD;
#endif

/** @brief Climbing mechanism command set. */
#ifndef HAVE_ENUM_CLIMBING_CMD_TYPE
#define HAVE_ENUM_CLIMBING_CMD_TYPE
typedef enum CLIMBING_CMD_TYPE
{
   CLIMBING_CMD_INIT_POSE=0, /* Keep the mechanism in the normal 20cm preparation pose. Maps to Climbing_Init_Pose_Start(). | */
   CLIMBING_CMD_PREPARE_40CM=1, /* Raise to the 40cm preparation pose. Maps to Climbing_Prepare_40cm(). | */
   CLIMBING_CMD_EXECUTE_UP_20CM=2, /* Execute 20cm climbing after the upper computer has moved the chassis to the edge. Maps to Climbing_Auto_Start_From_Touch_20cm(). | */
   CLIMBING_CMD_EXECUTE_UP_40CM=3, /* Execute 40cm climbing after the upper computer has moved the chassis to the edge. Maps to Climbing_Auto_Start_From_Touch_40cm(). | */
   CLIMBING_CMD_EXECUTE_DOWN_20CM=4, /* Execute 20cm descending after the upper computer has moved the chassis to the edge. Maps to Climbing_Descend_Auto_Start_20cm(). | */
   CLIMBING_CMD_TYPE_ENUM_END=5, /*  | */
} CLIMBING_CMD_TYPE;
#endif

/** @brief Weapon-head clamping command set. */
#ifndef HAVE_ENUM_CLAMPING_CMD_TYPE
#define HAVE_ENUM_CLAMPING_CMD_TYPE
typedef enum CLAMPING_CMD_TYPE
{
   CLAMPING_CMD_MOVE_TO_PARALLEL=0, /* Move the mechanism to the parallel/clamp-ready angle (90 deg). Maps to ClampingController::MoveToClampAngle(). | */
   CLAMPING_CMD_GRAB=1, /* Close the claw to clamp the weapon head. Maps to ClampingController::OpenSolenoid(). | */
   CLAMPING_CMD_MOVE_TO_RESET=2, /* Reset the mechanism angle back to 0 deg. Maps to ClampingController::MoveToResetAngle(). | */
   CLAMPING_CMD_RELEASE=3, /* Release the claw. Maps to ClampingController::ReleaseSolenoid(). | */
   CLAMPING_CMD_TYPE_ENUM_END=4, /*  | */
} CLAMPING_CMD_TYPE;
#endif

// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 1
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 1
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_arm_control.h"
#include "./mavlink_msg_chassis_velocity_cmd.h"
#include "./mavlink_msg_climbing_cmd.h"
#include "./mavlink_msg_clamping_cmd.h"

// base include



#if MAVLINK_R2_V2_XML_HASH == MAVLINK_PRIMARY_XML_HASH
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_ARM_CONTROL, MAVLINK_MESSAGE_INFO_CHASSIS_VELOCITY_CMD, MAVLINK_MESSAGE_INFO_CLIMBING_CMD, MAVLINK_MESSAGE_INFO_CLAMPING_CMD}
# define MAVLINK_MESSAGE_NAMES {{ "ARM_CONTROL", 200 }, { "CHASSIS_VELOCITY_CMD", 201 }, { "CLAMPING_CMD", 203 }, { "CLIMBING_CMD", 202 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_R2_V2_H
