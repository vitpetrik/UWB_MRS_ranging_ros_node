/**
 * @file uwb_range_node.hpp
 * @author Vit Petrik (petrivi2@fel.cvut.cz)
 * @brief 
 * @version 0.1
 * @date 2022-11-17
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _UWB_RANGE_NODE_H_
#define _UWB_RANGE_NODE_H_

#include <ros/ros.h>
#include <ros/package.h>

#include <unordered_map>

#include <mrs_modules_msgs/BacaProtocol.h>

#include <uwb_range/BeaconStamped.h>
#include <uwb_range/Beacon.h>

#include "protocol.h"

namespace uwb {
    class UwbRange {
        public:
            UwbRange(ros::NodeHandle nh);
            ~UwbRange();

        /**
         * Methods
        */

        void reset();

        private:

        /**
         * Variables
        */
            ros::NodeHandle nh;
            int preprocessing;

            int id;
            std::string uav_name;
            int enable_requests;
            std::string output_frame;
            std::unordered_map<uint16_t, uint64_t> ARP_table;

            ros::Time last_odom_time;
            ros::Time last_gps_time;

            double variance;

            uwb_range::Beacon beacon_msg;

        /** 
         * Publisher definitions
         */
            ros::Publisher baca_write;
            ros::Publisher range_out;
            ros::Publisher beacon_out;

        /** 
         * Subscribers definitions
         */
            ros::Subscriber baca_read;
            ros::Subscriber odometry_sub;
            ros::Subscriber gps_sub;

        /**
         * Timers
         */

            ros::Timer beacon_timer;
            ros::Timer ping_timer;

        /**
         * Methods
        */

       /**
        * @brief Deserialize and handle anchor message type
        * 
        * @param uwb_data  pointer to uwb_data_msg_t
        */
        void handle_anchor_msg(struct uwb_data_msg_t &uwb_data, ros::Time stamp);

       /**
        * @brief Deserialize and handle ros message type
        * 
        * @param uwb_data  pointer to uwb_data_msg_t
        */
        void handle_ros_msg(struct uwb_data_msg_t &uwb_data, ros::Time stamp);

        /**
         * @brief Send request message to UWB
         * 
         * @param target_mac L2 address of target
         * @param preprocessing preprocessing type
         */
        void request_ranging(uint16_t target_mac, int preprocessing);

        /** 
         * Callbacks
         */

        /**
         * @brief Receive message from baca node
         * 
         * @param serial_msg received message
         */
        void baca_read_cb(const mrs_modules_msgs::BacaProtocol serial_msg);

        /**
         * @brief Peridocally send beacon frame
         * 
         */
        void beacon_timer_cb(const ros::TimerEvent&);

        /**
         * @brief Send WHO_I_AM message to uwb
         * 
         */
        void ping_timer_cb(const ros::TimerEvent&);

        /**
         * @brief Odometry topic callback
         * 
         */
        void odometry_cb(const nav_msgs::OdometryConstPtr &msg);

        /**
         * @brief GPS topic callback
         * 
         */
        void gps_cb(const sensor_msgs::NavSatFixConstPtr &msg);
    };
}

#endif /* _UWB_RANGE_NODE_H_ */