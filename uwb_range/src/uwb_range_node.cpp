#include <ros/ros.h>
#include <ros/package.h>
#include <mrs_msgs/BacaProtocol.h>
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

#include "uwb_range_node.hpp"

namespace uwb
{
    UwbRange::UwbRange(ros::NodeHandle nh)
    {
        this->baca_write = nh.advertise<mrs_msgs::BacaProtocol>("baca_out", 1);
        this->baca_read = nh.subscribe("baca_in", 10, &UwbRange::baca_read_cb, this);
    }

    UwbRange::~UwbRange()
    {
    }

    void UwbRange::baca_read_cb(const mrs_msgs::BacaProtocol msg)
    {
        ROS_INFO("[UWB_RANGER]: received %ld bytes", msg.payload.size());
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