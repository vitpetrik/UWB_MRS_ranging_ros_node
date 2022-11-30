/**
 * @file uwb_range_node.cpp
 * @author Vit Petrik (petrivi2@fel.cvut.cz)
 * @brief
 * @version 0.1
 * @date 2022-11-17
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <mrs_msgs/BacaProtocol.h>
#include <cstdint>

#include "uwb_range_node.hpp"

enum
{
    WHO_I_AM,
    RADIO_CONFIG,
    RANGING_MODE,
    RANGING_RESULT,
    TRX_DATA
};

struct ranging_msg_t
{
    uint16_t source_mac;

    float range;
    float variance;
};

struct uwb_data_msg_t
{
    uint16_t source_mac;
    uint16_t destination_mac;

    uint8_t msg_type;
    uint8_t payload_size;
    uint8_t payload[128];
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

struct ros_msg_t
{
    uint8_t address;
    uint8_t mode;
    union
    {
        struct ros_id_msg_t id_msg;
        struct ros_radio_config_msg_t radio_config_msg;
        struct ros_ranging_mode_msg_t ranging_mode_msg;
        struct ranging_msg_t ranging_msg;
        struct uwb_data_msg_t uwb_data_msg;

    } data;
};

void serialize_ros(const struct ros_msg_t *msg, mrs_msgs::BacaProtocol &baca)
{
    baca.payload.push_back(msg->address);
    baca.payload.push_back(msg->mode);

    if (msg->mode == 'r')
        return;

    switch (msg->address)
    {
    case WHO_I_AM:
        baca.payload.insert(baca.payload.end(),
                            msg->data.id_msg.id,
                            msg->data.id_msg.id + sizeof(msg->data.id_msg.id));
        break;

    default:
        break;
    }
}

float ReverseFloat( const float inFloat )
{
   float retVal;
   char *floatToConvert = ( char* ) & inFloat;
   char *returnFloat = ( char* ) & retVal;

   // swap the bytes into a temporary buffer
   returnFloat[0] = floatToConvert[3];
   returnFloat[1] = floatToConvert[2];
   returnFloat[2] = floatToConvert[1];
   returnFloat[3] = floatToConvert[0];

   return retVal;
}

void deserialize_ros(struct ros_msg_t *msg, const mrs_msgs::BacaProtocol &baca)
{
    int index = 0;

    msg->address = baca.payload[index++];
    msg->mode = baca.payload[index++];

    if (msg->mode == 'r')
        return;

    switch (msg->address)
    {
    case WHO_I_AM:
        std::copy(baca.payload.begin() + index,
                  baca.payload.begin() + index + sizeof(msg->data.id_msg.id),
                  &msg->data.id_msg.id[0]);
        break;
    case RANGING_RESULT:
        msg->data.ranging_msg.range = 69;
        std::copy(baca.payload.begin() + index,
                  baca.payload.begin() + index + sizeof(msg->data.ranging_msg.source_mac),
                  &msg->data.ranging_msg.source_mac);
        index += sizeof(msg->data.ranging_msg.source_mac);
        std::copy(baca.payload.begin() + index,
                  baca.payload.begin() + index + sizeof(msg->data.ranging_msg.range),
                  &msg->data.ranging_msg.range);
        index += sizeof(msg->data.ranging_msg.range);
        std::copy(baca.payload.begin() + index,
                  baca.payload.begin() + index + sizeof(msg->data.ranging_msg.variance),
                  &msg->data.ranging_msg.variance);
        index += sizeof(msg->data.ranging_msg.variance);

        msg->data.ranging_msg.range = ReverseFloat(msg->data.ranging_msg.range);
        msg->data.ranging_msg.variance = ReverseFloat(msg->data.ranging_msg.variance);
        break;
    default:
        break;
    }
}

namespace uwb
{
    UwbRange::UwbRange(ros::NodeHandle nh)
    {
        this->baca_write = nh.advertise<mrs_msgs::BacaProtocol>("baca_out", 1);
        this->baca_read = nh.subscribe("baca_in", 10, &UwbRange::baca_read_cb, this);

        this->baca_timer = nh.createTimer(ros::Duration(1), &UwbRange::baca_timer_cb, this);

        return;
    }

    UwbRange::~UwbRange()
    {
    }

    void UwbRange::baca_read_cb(const mrs_msgs::BacaProtocol serial_msg)
    {
        struct ros_msg_t msg;
        deserialize_ros(&msg, serial_msg);

        switch (msg.address)
        {
        case WHO_I_AM:
            ROS_INFO("[UWB_RANGER]: Received WHO_I_AM %s", msg.data.id_msg.id);
            break;
        case RANGING_RESULT:
            ROS_INFO("[UWB_RANGER]: Received RANGING_RESULT 0x%X | %f m | %f",
                     msg.data.ranging_msg.source_mac,
                     msg.data.ranging_msg.range,
                     msg.data.ranging_msg.variance);
        default:
            break;
        }

        return;
    }

    void UwbRange::baca_timer_cb(const ros::TimerEvent &)
    {
        mrs_msgs::BacaProtocol serial_msg;

        serial_msg.stamp = ros::Time::now();

        struct ros_msg_t msg;
        msg.address = WHO_I_AM;
        msg.mode = 'r';

        serialize_ros(&msg, serial_msg);

        this->baca_write.publish(serial_msg);
        return;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uwb_range");
    ros::NodeHandle nh("~");
    uwb::UwbRange range(nh);

    ROS_INFO("[UWB_RANGER]: Node set");

    ros::spin();

    return 0;
}