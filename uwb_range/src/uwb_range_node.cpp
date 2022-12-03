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

#include "uwb_range_node.h"
#include "protocol.h"

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