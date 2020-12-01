#pragma once
// MESSAGE STATUSTEXT_LONG PACKING

#define MAVLINK_MSG_ID_STATUSTEXT_LONG 365

MAVPACKED(
typedef struct __mavlink_statustext_long_t {
 uint8_t severity; /*<  Severity of status. Relies on the definitions within RFC-5424.*/
 char text[254]; /*<  Status text message, without null termination character.*/
}) mavlink_statustext_long_t;

#define MAVLINK_MSG_ID_STATUSTEXT_LONG_LEN 255
#define MAVLINK_MSG_ID_STATUSTEXT_LONG_MIN_LEN 255
#define MAVLINK_MSG_ID_365_LEN 255
#define MAVLINK_MSG_ID_365_MIN_LEN 255

#define MAVLINK_MSG_ID_STATUSTEXT_LONG_CRC 36
#define MAVLINK_MSG_ID_365_CRC 36

#define MAVLINK_MSG_STATUSTEXT_LONG_FIELD_TEXT_LEN 254

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_STATUSTEXT_LONG { \
    365, \
    "STATUSTEXT_LONG", \
    2, \
    {  { "severity", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_statustext_long_t, severity) }, \
         { "text", NULL, MAVLINK_TYPE_CHAR, 254, 1, offsetof(mavlink_statustext_long_t, text) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_STATUSTEXT_LONG { \
    "STATUSTEXT_LONG", \
    2, \
    {  { "severity", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_statustext_long_t, severity) }, \
         { "text", NULL, MAVLINK_TYPE_CHAR, 254, 1, offsetof(mavlink_statustext_long_t, text) }, \
         } \
}
#endif

/**
 * @brief Pack a statustext_long message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param severity  Severity of status. Relies on the definitions within RFC-5424.
 * @param text  Status text message, without null termination character.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_statustext_long_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t severity, const char *text)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STATUSTEXT_LONG_LEN];
    _mav_put_uint8_t(buf, 0, severity);
    _mav_put_char_array(buf, 1, text, 254);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STATUSTEXT_LONG_LEN);
#else
    mavlink_statustext_long_t packet;
    packet.severity = severity;
    mav_array_memcpy(packet.text, text, sizeof(char)*254);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STATUSTEXT_LONG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STATUSTEXT_LONG;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_STATUSTEXT_LONG_MIN_LEN, MAVLINK_MSG_ID_STATUSTEXT_LONG_LEN, MAVLINK_MSG_ID_STATUSTEXT_LONG_CRC);
}

/**
 * @brief Pack a statustext_long message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param severity  Severity of status. Relies on the definitions within RFC-5424.
 * @param text  Status text message, without null termination character.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_statustext_long_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t severity,const char *text)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STATUSTEXT_LONG_LEN];
    _mav_put_uint8_t(buf, 0, severity);
    _mav_put_char_array(buf, 1, text, 254);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STATUSTEXT_LONG_LEN);
#else
    mavlink_statustext_long_t packet;
    packet.severity = severity;
    mav_array_memcpy(packet.text, text, sizeof(char)*254);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STATUSTEXT_LONG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STATUSTEXT_LONG;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_STATUSTEXT_LONG_MIN_LEN, MAVLINK_MSG_ID_STATUSTEXT_LONG_LEN, MAVLINK_MSG_ID_STATUSTEXT_LONG_CRC);
}

/**
 * @brief Encode a statustext_long struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param statustext_long C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_statustext_long_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_statustext_long_t* statustext_long)
{
    return mavlink_msg_statustext_long_pack(system_id, component_id, msg, statustext_long->severity, statustext_long->text);
}

/**
 * @brief Encode a statustext_long struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param statustext_long C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_statustext_long_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_statustext_long_t* statustext_long)
{
    return mavlink_msg_statustext_long_pack_chan(system_id, component_id, chan, msg, statustext_long->severity, statustext_long->text);
}

/**
 * @brief Send a statustext_long message
 * @param chan MAVLink channel to send the message
 *
 * @param severity  Severity of status. Relies on the definitions within RFC-5424.
 * @param text  Status text message, without null termination character.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_statustext_long_send(mavlink_channel_t chan, uint8_t severity, const char *text)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STATUSTEXT_LONG_LEN];
    _mav_put_uint8_t(buf, 0, severity);
    _mav_put_char_array(buf, 1, text, 254);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATUSTEXT_LONG, buf, MAVLINK_MSG_ID_STATUSTEXT_LONG_MIN_LEN, MAVLINK_MSG_ID_STATUSTEXT_LONG_LEN, MAVLINK_MSG_ID_STATUSTEXT_LONG_CRC);
#else
    mavlink_statustext_long_t packet;
    packet.severity = severity;
    mav_array_memcpy(packet.text, text, sizeof(char)*254);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATUSTEXT_LONG, (const char *)&packet, MAVLINK_MSG_ID_STATUSTEXT_LONG_MIN_LEN, MAVLINK_MSG_ID_STATUSTEXT_LONG_LEN, MAVLINK_MSG_ID_STATUSTEXT_LONG_CRC);
#endif
}

/**
 * @brief Send a statustext_long message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_statustext_long_send_struct(mavlink_channel_t chan, const mavlink_statustext_long_t* statustext_long)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_statustext_long_send(chan, statustext_long->severity, statustext_long->text);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATUSTEXT_LONG, (const char *)statustext_long, MAVLINK_MSG_ID_STATUSTEXT_LONG_MIN_LEN, MAVLINK_MSG_ID_STATUSTEXT_LONG_LEN, MAVLINK_MSG_ID_STATUSTEXT_LONG_CRC);
#endif
}

#if MAVLINK_MSG_ID_STATUSTEXT_LONG_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_statustext_long_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t severity, const char *text)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, severity);
    _mav_put_char_array(buf, 1, text, 254);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATUSTEXT_LONG, buf, MAVLINK_MSG_ID_STATUSTEXT_LONG_MIN_LEN, MAVLINK_MSG_ID_STATUSTEXT_LONG_LEN, MAVLINK_MSG_ID_STATUSTEXT_LONG_CRC);
#else
    mavlink_statustext_long_t *packet = (mavlink_statustext_long_t *)msgbuf;
    packet->severity = severity;
    mav_array_memcpy(packet->text, text, sizeof(char)*254);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATUSTEXT_LONG, (const char *)packet, MAVLINK_MSG_ID_STATUSTEXT_LONG_MIN_LEN, MAVLINK_MSG_ID_STATUSTEXT_LONG_LEN, MAVLINK_MSG_ID_STATUSTEXT_LONG_CRC);
#endif
}
#endif

#endif

// MESSAGE STATUSTEXT_LONG UNPACKING


/**
 * @brief Get field severity from statustext_long message
 *
 * @return  Severity of status. Relies on the definitions within RFC-5424.
 */
static inline uint8_t mavlink_msg_statustext_long_get_severity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field text from statustext_long message
 *
 * @return  Status text message, without null termination character.
 */
static inline uint16_t mavlink_msg_statustext_long_get_text(const mavlink_message_t* msg, char *text)
{
    return _MAV_RETURN_char_array(msg, text, 254,  1);
}

/**
 * @brief Decode a statustext_long message into a struct
 *
 * @param msg The message to decode
 * @param statustext_long C-struct to decode the message contents into
 */
static inline void mavlink_msg_statustext_long_decode(const mavlink_message_t* msg, mavlink_statustext_long_t* statustext_long)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    statustext_long->severity = mavlink_msg_statustext_long_get_severity(msg);
    mavlink_msg_statustext_long_get_text(msg, statustext_long->text);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_STATUSTEXT_LONG_LEN? msg->len : MAVLINK_MSG_ID_STATUSTEXT_LONG_LEN;
        memset(statustext_long, 0, MAVLINK_MSG_ID_STATUSTEXT_LONG_LEN);
    memcpy(statustext_long, _MAV_PAYLOAD(msg), len);
#endif
}
