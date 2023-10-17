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
#include <ros/serialization.h>

#include <tf/tf.h>


#include <math.h>
#include <cmath>
#include <random>

#include <mrs_msgs/RangeWithCovarianceArrayStamped.h>
#include <mrs_modules_msgs/BacaProtocol.h>
#include <mrs_msgs/RangeWithVar.h>

#include <geometry_msgs/Point.h>
#include <sensor_msgs/NavSatFix.h>
#include <mrs_msgs/NavSatFixArrayStamped.h>
#include <nav_msgs/Odometry.h>

#include <uwb_range/BeaconStamped.h>
#include <uwb_range/Beacon.h>

#include <mrs_lib/param_loader.h>

#include <cstdint>
#include "msg.pb.h"

#include <sstream>
#include <iostream>

#include "uwb_range_node.h"
#include "protocol.h"

namespace uwb
{
    UwbRange::UwbRange(ros::NodeHandle nh)
    {
        this->nh = nh;

        mrs_lib::ParamLoader param_loader(nh, "UWB range");

        param_loader.loadParam("uwb_id", this->id);
        param_loader.loadParam("enable_requests", this->enable_requests);
        param_loader.loadParam("output_frame", this->output_frame);
        param_loader.loadParam("variance", this->variance);
        param_loader.loadParam("uav_name", this->uav_name);

        this->beacon_msg.id = this->id;
        this->beacon_msg.uav_name = this->uav_name;
        this->beacon_msg.uav_type = 1;

        this->beacon_msg.ALT = nan("");
        this->beacon_msg.LAT = nan("");
        this->beacon_msg.LON = nan("");
        this->beacon_msg.heading = nan("");

        this->baca_write = nh.advertise<mrs_modules_msgs::BacaProtocol>("baca_out", 1);
        this->baca_read = nh.subscribe("baca_in", 10, &UwbRange::baca_read_cb, this);

        this->gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("gps_in", 10, &UwbRange::gps_cb, this);
        this->odometry_sub = nh.subscribe<nav_msgs::Odometry>("odometry_in", 10, &UwbRange::odometry_cb, this);

        this->range_out = nh.advertise<mrs_msgs::RangeWithCovarianceArrayStamped>("range_out", 1);
        this->beacon_out = nh.advertise<uwb_range::BeaconStamped>("beacon_out", 10, this);

        // choose random period for beacon timer
        // to prevent some unwanted synchronizations

        std::random_device rd;
        std::mt19937 e2(rd());
        std::uniform_real_distribution<> beacon_period(0.8, 1.2);

        this->beacon_timer = nh.createTimer(ros::Duration(beacon_period(e2)), &UwbRange::beacon_timer_cb, this);
        this->ping_timer = nh.createTimer(ros::Duration(1), &UwbRange::ping_timer_cb, this);

        return;
    }

    UwbRange::~UwbRange()
    {
        this->baca_read.shutdown();
        this->reset();

        return;
    }

    void UwbRange::reset()
    {
        ROS_INFO("[UWB_RANGER]: Resetting the device");
        mrs_modules_msgs::BacaProtocol serial_msg;
        serial_msg.stamp = ros::Time::now();

        struct ros_msg_t msg;
        msg.address = RESET;
        msg.mode = 'w';
        msg.data.reset = 0xff;

        serialize_ros(&msg, serial_msg);

        this->baca_write.publish(serial_msg);
        return;
    }

    /**
     * @brief Deserialize and handle anchor message type
     *
     * @param uwb_data  pointer to uwb_data_msg_t
     */
    void UwbRange::handle_anchor_msg(struct uwb_data_msg_t &uwb_data, ros::Time stamp)
    {
        struct anchor_msg_t anchor_msg;
        deserialize_anchor_msg(&anchor_msg, uwb_data.payload);

        if (anchor_msg.data.anchor_beacon.capabilities != RDEV)
            return;

        this->nh.getParam("preprocessing", this->preprocessing);
        this->request_ranging(uwb_data.source_mac, this->preprocessing);

        return;
    }

    /**
     * @brief Deserialize and handle ros message type
     *
     * @param uwb_data  pointer to uwb_data_msg_t
     */
    void UwbRange::handle_ros_msg(struct uwb_data_msg_t &uwb_data, ros::Time stamp)
    {
        uwb_range::Beacon beacon_msg;

        ros::serialization::IStream stream(uwb_data.payload, uwb_data.payload_size);
        ros::serialization::Serializer<uwb_range::Beacon>::read(stream, beacon_msg);

        ROS_INFO("[UWB_RANGER]: Beacon RX from %s", beacon_msg.uav_name.c_str());

        this->ARP_table[uwb_data.source_mac] = beacon_msg.id;

        uwb_range::BeaconStamped beacon_stamped;
        beacon_stamped.beacon = beacon_msg;
        beacon_stamped.header.stamp = stamp;
        beacon_stamped.header.frame_id = this->uav_name + "/utm_origin";

        this->beacon_out.publish(beacon_stamped);

        if (!enable_requests)
            return;

        this->nh.getParam("preprocessing", this->preprocessing);
        this->request_ranging(uwb_data.source_mac, this->preprocessing);

        return;
    }

    /**
     * @brief Send request message to UWB
     *
     * @param target_mac L2 address of target
     * @param preprocessing preprocessing type
     */
    void UwbRange::request_ranging(uint16_t target_mac, int preprocessing)
    {
        mrs_modules_msgs::BacaProtocol baca_out;
        struct ros_msg_t msg;

        msg.address = REQUEST_RANGING;
        msg.data.request_ranging.target_id = target_mac;
        msg.data.request_ranging.preprocessing = this->preprocessing;

        serialize_ros(&msg, baca_out);
        this->baca_write.publish(baca_out);

        return;
    }

    /**
     * @brief Receive message from baca node
     *
     * @param serial_msg received message
     */
    void UwbRange::baca_read_cb(const mrs_modules_msgs::BacaProtocol serial_msg)
    {
        struct ros_msg_t msg;
        deserialize_ros(&msg, serial_msg);

        mrs_modules_msgs::BacaProtocol out_msg;
        struct anchor_msg_t anchor_msg;

        switch (msg.address)
        {
        case WHO_I_AM:
            ROS_INFO("[UWB_RANGER]: Received WHO_I_AM %s", msg.data.id_msg.id);
            break;
        case RANGING_RESULT:
        {
            struct ranging_msg_t &ranging_msg = msg.data.ranging_msg;

            ROS_INFO_THROTTLE(0.5, "[UWB_RANGER]: Received RANGING_RESULT 0x%X | %.1f m | %f | %.1f dBm | %.1f dBm",
                              ranging_msg.source_mac,
                              ranging_msg.range,
                              ranging_msg.variance,
                              ranging_msg.power_a,
                              ranging_msg.power_b);

            if (not this->ARP_table.count(ranging_msg.source_mac))
            {
                ROS_INFO("[UWB_RANGER]: MAC address 0x%X not found in ARP table", ranging_msg.source_mac);
            }

            uint64_t source_id = this->ARP_table[ranging_msg.source_mac];

            mrs_msgs::RangeWithCovarianceArrayStamped out_msg;
            mrs_msgs::RangeWithCovarianceIdentified range;

            out_msg.header.stamp = serial_msg.stamp;
            out_msg.header.frame_id = this->output_frame;

            range.id = source_id;
            range.variance = this->variance;
            range.range.range = ranging_msg.range;
            range.power_a = ranging_msg.power_a;
            range.power_b = ranging_msg.power_b;

            range.range.field_of_view = 2 * M_PI;
            range.range.radiation_type = 3;
            range.range.min_range = 0;
            range.range.max_range = 100;

            out_msg.ranges.push_back(range);

            this->range_out.publish(out_msg);
            break;
        }
        case TRX_DATA:
            ROS_INFO("[UWB_RANGER]: Received message from 0x%X", msg.data.uwb_data_msg.source_mac);

            switch (msg.data.uwb_data_msg.msg_type)
            {
            case ANCHOR_TYPE:
                handle_anchor_msg(msg.data.uwb_data_msg, serial_msg.stamp);
                break;
            case ROS_TYPE:
                handle_ros_msg(msg.data.uwb_data_msg, serial_msg.stamp);
                break;
            default:
                break;
            }
            break;
        case RESET:
            ROS_INFO("[UWB_RANGER]: Setting ROS control");

            msg.address = ROS_CONTROL;
            msg.mode = 'w';
            msg.data.control = ROS;

            serialize_ros(&msg, out_msg);
            this->baca_write.publish(out_msg);
            break;
        default:
            break;
        }

        return;
    }

    /**
     * @brief Peridocally send beacon frame
     *
     */
    void UwbRange::beacon_timer_cb(const ros::TimerEvent &)
    {
        struct ros_msg_t msg;
        mrs_modules_msgs::BacaProtocol baca_out;

        if(ros::Time::now() - this->last_odom_time > ros::Duration(1))
        {
            this->beacon_msg.ALT = nan("");
            this->beacon_msg.heading = nan("");
        }

        if(ros::Time::now() - this->last_gps_time > ros::Duration(1))
        {
            this->beacon_msg.LAT = nan("");
            this->beacon_msg.LON = nan("");
        }

        uint32_t serial_size = ros::serialization::serializationLength(this->beacon_msg);
        msg.data.uwb_data_msg.payload_size = serial_size;

        ros::serialization::OStream stream(msg.data.uwb_data_msg.payload, serial_size);
        ros::serialization::serialize(stream, this->beacon_msg);

        msg.address = TRX_DATA;
        msg.mode = 'w';
        msg.data.uwb_data_msg.destination_mac = 0xffff;
        msg.data.uwb_data_msg.msg_type = ROS_TYPE;

        serialize_ros(&msg, baca_out);
        this->baca_write.publish(baca_out);

        return;
    }

    /**
     * @brief Sens WHO_I_AM message to uwb
     *
     */
    void UwbRange::ping_timer_cb(const ros::TimerEvent &)
    {
        struct ros_msg_t msg;
        mrs_modules_msgs::BacaProtocol baca_out;
        ROS_INFO("[UWB_RANGER]: Asking for WHO_I_AM");

        msg.address = WHO_I_AM;
        msg.mode = 'r';

        serialize_ros(&msg, baca_out);
        this->baca_write.publish(baca_out);
    }

    /**
     * @brief Odometry topic callback
     *
     */
    void UwbRange::odometry_cb(const nav_msgs::OdometryConstPtr &msg)
    {
        Eigen::Vector3d x = Eigen::Vector3d(1, 0, 0);
        Eigen::Quaterniond quat = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                                     msg->pose.pose.orientation.x,
                                                     msg->pose.pose.orientation.y,
                                                     msg->pose.pose.orientation.z);

        x = quat * x;

        this->beacon_msg.heading = atan2(x(1), x(0));
        this->beacon_msg.ALT = msg->pose.pose.position.z;

        this->last_odom_time = msg->header.stamp;

        return;
    }

    /**
     * @brief GPS topic callback
     *
     */
    void UwbRange::gps_cb(const sensor_msgs::NavSatFixConstPtr &msg)
    {
        if(msg->status.status < 0)
            return;

        this->beacon_msg.LAT = msg->latitude;
        this->beacon_msg.LON = msg->longitude;

        this->last_gps_time = msg->header.stamp;

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