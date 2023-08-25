/**
 * @file protocol.h
 * @author Vit Petrik (petrivi2@fel.cvut.cz)
 * @brief Serialization and deserialization of protocol used to communicate between nRF and ROS
 * @version 0.1
 * @date 2022-12-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include <cstdint>
#include <mrs_msgs/BacaProtocol.h>

typedef enum {
    UNDETERMINED,
    STANDALONE,
    ROS
} control_t;

struct ranging_msg_t
{
    uint16_t source_mac;

    float range;
    float variance;

    float raw;

    float power_a;
    float power_b;
};

struct uwb_data_msg_t
{
    uint16_t source_mac;
    uint16_t destination_mac;

    uint8_t msg_type;
    uint8_t payload_size;
    uint8_t payload[128];
};

enum PACKET_MSG_TYPES
{
    ROS_TYPE,
    ANCHOR_TYPE,
    RANGING_TYPE
};

enum address_t
{
    WHO_I_AM,
    RADIO_CONFIG,
    RANGING_MODE,
    RANGING_RESULT,
    TRX_DATA,
    RESET,
    ROS_CONTROL,
    REQUEST_RANGING,
};

struct ros_id_msg_t
{
    char id[16];
};

struct ros_radio_config_msg_t
{
};

struct ros_ranging_mode_msg_t
{
};

struct request_ranging_t
{
    uint16_t target_id;
    uint8_t preprocessing;
};

struct ros_msg_t
{
    uint8_t address;
    char mode;
    union
    {
        struct ros_id_msg_t id_msg;
        struct ros_radio_config_msg_t radio_config_msg;
        struct ros_ranging_mode_msg_t ranging_mode_msg;
        struct ranging_msg_t ranging_msg;
        struct uwb_data_msg_t uwb_data_msg;
        uint8_t reset;
        uint8_t control;
        struct request_ranging_t request_ranging;
    } data;
};

enum address_anchor_t
{
    ANCHOR_BEACON,
};

enum capabilities_t
{
    NONE,
    RDEV
};

struct anchor_beacon_t
{
    uint8_t capabilities;
};

struct anchor_msg_t
{
    uint8_t address;
    char mode;

    union
    {
        struct anchor_beacon_t anchor_beacon;
    } data;
};

/**
 * @brief serialize ros_msg_t to baca protocol struct
 * 
 * @param msg reference to an struct ros_msg_t
 * @param baca refernce to baca protocol structure
 */
void serialize_ros(const struct ros_msg_t *msg, mrs_msgs::BacaProtocol &baca);

/**
 * @brief deserialize baca protocol msg to ros_msg_t
 * 
 * @param msg reference to an struct ros_msg_t
 * @param baca refernce to baca protocol structure
 */
void deserialize_ros(struct ros_msg_t *msg, const mrs_msgs::BacaProtocol &baca);

/**
 * @brief deserialize baca protocol msg to anchor_msg_t
 * 
 * @param msg reference to an struct anchor_msg_t
 * @param baca refernce to baca protocol structure
*/
void deserialize_anchor_msg(struct anchor_msg_t *msg, uint8_t *buffer);

#endif