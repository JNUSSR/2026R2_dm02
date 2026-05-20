#pragma once
// MESSAGE CHASSIS_VELOCITY_CMD PACKING

#define MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD 201


typedef struct __mavlink_chassis_velocity_cmd_t {
 float vx; /*< [m/s] X-axis target velocity*/
 float vy; /*< [m/s] Y-axis target velocity*/
 float vw; /*< [rad/s] Rotation target velocity*/
} mavlink_chassis_velocity_cmd_t;

#define MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_LEN 12
#define MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_MIN_LEN 12
#define MAVLINK_MSG_ID_201_LEN 12
#define MAVLINK_MSG_ID_201_MIN_LEN 12

#define MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_CRC 211
#define MAVLINK_MSG_ID_201_CRC 211



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CHASSIS_VELOCITY_CMD { \
    201, \
    "CHASSIS_VELOCITY_CMD", \
    3, \
    {  { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_chassis_velocity_cmd_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_chassis_velocity_cmd_t, vy) }, \
         { "vw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_chassis_velocity_cmd_t, vw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CHASSIS_VELOCITY_CMD { \
    "CHASSIS_VELOCITY_CMD", \
    3, \
    {  { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_chassis_velocity_cmd_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_chassis_velocity_cmd_t, vy) }, \
         { "vw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_chassis_velocity_cmd_t, vw) }, \
         } \
}
#endif

/**
 * @brief Pack a chassis_velocity_cmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param vx [m/s] X-axis target velocity
 * @param vy [m/s] Y-axis target velocity
 * @param vw [rad/s] Rotation target velocity
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chassis_velocity_cmd_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float vx, float vy, float vw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_LEN];
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, vy);
    _mav_put_float(buf, 8, vw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_LEN);
#else
    mavlink_chassis_velocity_cmd_t packet;
    packet.vx = vx;
    packet.vy = vy;
    packet.vw = vw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_MIN_LEN, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_LEN, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_CRC);
}

/**
 * @brief Pack a chassis_velocity_cmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param vx [m/s] X-axis target velocity
 * @param vy [m/s] Y-axis target velocity
 * @param vw [rad/s] Rotation target velocity
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chassis_velocity_cmd_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float vx, float vy, float vw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_LEN];
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, vy);
    _mav_put_float(buf, 8, vw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_LEN);
#else
    mavlink_chassis_velocity_cmd_t packet;
    packet.vx = vx;
    packet.vy = vy;
    packet.vw = vw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_MIN_LEN, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_LEN, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_MIN_LEN, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_LEN);
#endif
}

/**
 * @brief Pack a chassis_velocity_cmd message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vx [m/s] X-axis target velocity
 * @param vy [m/s] Y-axis target velocity
 * @param vw [rad/s] Rotation target velocity
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chassis_velocity_cmd_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float vx,float vy,float vw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_LEN];
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, vy);
    _mav_put_float(buf, 8, vw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_LEN);
#else
    mavlink_chassis_velocity_cmd_t packet;
    packet.vx = vx;
    packet.vy = vy;
    packet.vw = vw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_MIN_LEN, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_LEN, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_CRC);
}

/**
 * @brief Encode a chassis_velocity_cmd struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param chassis_velocity_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chassis_velocity_cmd_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_chassis_velocity_cmd_t* chassis_velocity_cmd)
{
    return mavlink_msg_chassis_velocity_cmd_pack(system_id, component_id, msg, chassis_velocity_cmd->vx, chassis_velocity_cmd->vy, chassis_velocity_cmd->vw);
}

/**
 * @brief Encode a chassis_velocity_cmd struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param chassis_velocity_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chassis_velocity_cmd_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_chassis_velocity_cmd_t* chassis_velocity_cmd)
{
    return mavlink_msg_chassis_velocity_cmd_pack_chan(system_id, component_id, chan, msg, chassis_velocity_cmd->vx, chassis_velocity_cmd->vy, chassis_velocity_cmd->vw);
}

/**
 * @brief Encode a chassis_velocity_cmd struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param chassis_velocity_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chassis_velocity_cmd_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_chassis_velocity_cmd_t* chassis_velocity_cmd)
{
    return mavlink_msg_chassis_velocity_cmd_pack_status(system_id, component_id, _status, msg,  chassis_velocity_cmd->vx, chassis_velocity_cmd->vy, chassis_velocity_cmd->vw);
}

/**
 * @brief Send a chassis_velocity_cmd message
 * @param chan MAVLink channel to send the message
 *
 * @param vx [m/s] X-axis target velocity
 * @param vy [m/s] Y-axis target velocity
 * @param vw [rad/s] Rotation target velocity
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_chassis_velocity_cmd_send(mavlink_channel_t chan, float vx, float vy, float vw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_LEN];
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, vy);
    _mav_put_float(buf, 8, vw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD, buf, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_MIN_LEN, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_LEN, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_CRC);
#else
    mavlink_chassis_velocity_cmd_t packet;
    packet.vx = vx;
    packet.vy = vy;
    packet.vw = vw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD, (const char *)&packet, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_MIN_LEN, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_LEN, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_CRC);
#endif
}

/**
 * @brief Send a chassis_velocity_cmd message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_chassis_velocity_cmd_send_struct(mavlink_channel_t chan, const mavlink_chassis_velocity_cmd_t* chassis_velocity_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_chassis_velocity_cmd_send(chan, chassis_velocity_cmd->vx, chassis_velocity_cmd->vy, chassis_velocity_cmd->vw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD, (const char *)chassis_velocity_cmd, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_MIN_LEN, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_LEN, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_CRC);
#endif
}

#if MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_chassis_velocity_cmd_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float vx, float vy, float vw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, vy);
    _mav_put_float(buf, 8, vw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD, buf, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_MIN_LEN, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_LEN, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_CRC);
#else
    mavlink_chassis_velocity_cmd_t *packet = (mavlink_chassis_velocity_cmd_t *)msgbuf;
    packet->vx = vx;
    packet->vy = vy;
    packet->vw = vw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD, (const char *)packet, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_MIN_LEN, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_LEN, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_CRC);
#endif
}
#endif

#endif

// MESSAGE CHASSIS_VELOCITY_CMD UNPACKING


/**
 * @brief Get field vx from chassis_velocity_cmd message
 *
 * @return [m/s] X-axis target velocity
 */
static inline float mavlink_msg_chassis_velocity_cmd_get_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field vy from chassis_velocity_cmd message
 *
 * @return [m/s] Y-axis target velocity
 */
static inline float mavlink_msg_chassis_velocity_cmd_get_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field vw from chassis_velocity_cmd message
 *
 * @return [rad/s] Rotation target velocity
 */
static inline float mavlink_msg_chassis_velocity_cmd_get_vw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a chassis_velocity_cmd message into a struct
 *
 * @param msg The message to decode
 * @param chassis_velocity_cmd C-struct to decode the message contents into
 */
static inline void mavlink_msg_chassis_velocity_cmd_decode(const mavlink_message_t* msg, mavlink_chassis_velocity_cmd_t* chassis_velocity_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    chassis_velocity_cmd->vx = mavlink_msg_chassis_velocity_cmd_get_vx(msg);
    chassis_velocity_cmd->vy = mavlink_msg_chassis_velocity_cmd_get_vy(msg);
    chassis_velocity_cmd->vw = mavlink_msg_chassis_velocity_cmd_get_vw(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_LEN? msg->len : MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_LEN;
        memset(chassis_velocity_cmd, 0, MAVLINK_MSG_ID_CHASSIS_VELOCITY_CMD_LEN);
    memcpy(chassis_velocity_cmd, _MAV_PAYLOAD(msg), len);
#endif
}
