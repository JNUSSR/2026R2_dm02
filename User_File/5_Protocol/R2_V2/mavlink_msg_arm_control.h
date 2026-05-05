#pragma once
// MESSAGE ARM_CONTROL PACKING

#define MAVLINK_MSG_ID_ARM_CONTROL 200


typedef struct __mavlink_arm_control_t {
 uint8_t action_cmd; /*<  Command ID to execute*/
} mavlink_arm_control_t;

#define MAVLINK_MSG_ID_ARM_CONTROL_LEN 1
#define MAVLINK_MSG_ID_ARM_CONTROL_MIN_LEN 1
#define MAVLINK_MSG_ID_200_LEN 1
#define MAVLINK_MSG_ID_200_MIN_LEN 1

#define MAVLINK_MSG_ID_ARM_CONTROL_CRC 230
#define MAVLINK_MSG_ID_200_CRC 230



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ARM_CONTROL { \
    200, \
    "ARM_CONTROL", \
    1, \
    {  { "action_cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_arm_control_t, action_cmd) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ARM_CONTROL { \
    "ARM_CONTROL", \
    1, \
    {  { "action_cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_arm_control_t, action_cmd) }, \
         } \
}
#endif

/**
 * @brief Pack a arm_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param action_cmd  Command ID to execute
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_arm_control_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t action_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ARM_CONTROL_LEN];
    _mav_put_uint8_t(buf, 0, action_cmd);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARM_CONTROL_LEN);
#else
    mavlink_arm_control_t packet;
    packet.action_cmd = action_cmd;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARM_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ARM_CONTROL;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ARM_CONTROL_MIN_LEN, MAVLINK_MSG_ID_ARM_CONTROL_LEN, MAVLINK_MSG_ID_ARM_CONTROL_CRC);
}

/**
 * @brief Pack a arm_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param action_cmd  Command ID to execute
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_arm_control_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t action_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ARM_CONTROL_LEN];
    _mav_put_uint8_t(buf, 0, action_cmd);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARM_CONTROL_LEN);
#else
    mavlink_arm_control_t packet;
    packet.action_cmd = action_cmd;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARM_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ARM_CONTROL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ARM_CONTROL_MIN_LEN, MAVLINK_MSG_ID_ARM_CONTROL_LEN, MAVLINK_MSG_ID_ARM_CONTROL_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ARM_CONTROL_MIN_LEN, MAVLINK_MSG_ID_ARM_CONTROL_LEN);
#endif
}

/**
 * @brief Pack a arm_control message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param action_cmd  Command ID to execute
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_arm_control_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t action_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ARM_CONTROL_LEN];
    _mav_put_uint8_t(buf, 0, action_cmd);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARM_CONTROL_LEN);
#else
    mavlink_arm_control_t packet;
    packet.action_cmd = action_cmd;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARM_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ARM_CONTROL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ARM_CONTROL_MIN_LEN, MAVLINK_MSG_ID_ARM_CONTROL_LEN, MAVLINK_MSG_ID_ARM_CONTROL_CRC);
}

/**
 * @brief Encode a arm_control struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param arm_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_arm_control_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_arm_control_t* arm_control)
{
    return mavlink_msg_arm_control_pack(system_id, component_id, msg, arm_control->action_cmd);
}

/**
 * @brief Encode a arm_control struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param arm_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_arm_control_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_arm_control_t* arm_control)
{
    return mavlink_msg_arm_control_pack_chan(system_id, component_id, chan, msg, arm_control->action_cmd);
}

/**
 * @brief Encode a arm_control struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param arm_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_arm_control_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_arm_control_t* arm_control)
{
    return mavlink_msg_arm_control_pack_status(system_id, component_id, _status, msg,  arm_control->action_cmd);
}

/**
 * @brief Send a arm_control message
 * @param chan MAVLink channel to send the message
 *
 * @param action_cmd  Command ID to execute
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_arm_control_send(mavlink_channel_t chan, uint8_t action_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ARM_CONTROL_LEN];
    _mav_put_uint8_t(buf, 0, action_cmd);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARM_CONTROL, buf, MAVLINK_MSG_ID_ARM_CONTROL_MIN_LEN, MAVLINK_MSG_ID_ARM_CONTROL_LEN, MAVLINK_MSG_ID_ARM_CONTROL_CRC);
#else
    mavlink_arm_control_t packet;
    packet.action_cmd = action_cmd;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARM_CONTROL, (const char *)&packet, MAVLINK_MSG_ID_ARM_CONTROL_MIN_LEN, MAVLINK_MSG_ID_ARM_CONTROL_LEN, MAVLINK_MSG_ID_ARM_CONTROL_CRC);
#endif
}

/**
 * @brief Send a arm_control message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_arm_control_send_struct(mavlink_channel_t chan, const mavlink_arm_control_t* arm_control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_arm_control_send(chan, arm_control->action_cmd);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARM_CONTROL, (const char *)arm_control, MAVLINK_MSG_ID_ARM_CONTROL_MIN_LEN, MAVLINK_MSG_ID_ARM_CONTROL_LEN, MAVLINK_MSG_ID_ARM_CONTROL_CRC);
#endif
}

#if MAVLINK_MSG_ID_ARM_CONTROL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_arm_control_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t action_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, action_cmd);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARM_CONTROL, buf, MAVLINK_MSG_ID_ARM_CONTROL_MIN_LEN, MAVLINK_MSG_ID_ARM_CONTROL_LEN, MAVLINK_MSG_ID_ARM_CONTROL_CRC);
#else
    mavlink_arm_control_t *packet = (mavlink_arm_control_t *)msgbuf;
    packet->action_cmd = action_cmd;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARM_CONTROL, (const char *)packet, MAVLINK_MSG_ID_ARM_CONTROL_MIN_LEN, MAVLINK_MSG_ID_ARM_CONTROL_LEN, MAVLINK_MSG_ID_ARM_CONTROL_CRC);
#endif
}
#endif

#endif

// MESSAGE ARM_CONTROL UNPACKING


/**
 * @brief Get field action_cmd from arm_control message
 *
 * @return  Command ID to execute
 */
static inline uint8_t mavlink_msg_arm_control_get_action_cmd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a arm_control message into a struct
 *
 * @param msg The message to decode
 * @param arm_control C-struct to decode the message contents into
 */
static inline void mavlink_msg_arm_control_decode(const mavlink_message_t* msg, mavlink_arm_control_t* arm_control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    arm_control->action_cmd = mavlink_msg_arm_control_get_action_cmd(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ARM_CONTROL_LEN? msg->len : MAVLINK_MSG_ID_ARM_CONTROL_LEN;
        memset(arm_control, 0, MAVLINK_MSG_ID_ARM_CONTROL_LEN);
    memcpy(arm_control, _MAV_PAYLOAD(msg), len);
#endif
}
