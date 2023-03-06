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

#include <mrs_msgs/BacaProtocol.h>

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
            std::string output_frame;
            std::unordered_map<uint16_t, uint64_t> ARP_table;

            double variance;

        /** 
         * Publisher definitions
         */
            ros::Publisher baca_write;
            ros::Publisher range_out;

        /** 
         * Subscribers definitions
         */
            ros::Subscriber baca_read;

        /**
         * Timers
         */

            ros::Timer beacon_timer;

        /**
         * Methods
        */

       /**
        * @brief Deserialize and handle anchor message type
        * 
        * @param uwb_data  pointer to uwb_data_msg_t
        */
        void handle_anchor_msg(struct uwb_data_msg_t &uwb_data);

       /**
        * @brief Deserialize and handle ros message type
        * 
        * @param uwb_data  pointer to uwb_data_msg_t
        */
        void handle_ros_msg(struct uwb_data_msg_t &uwb_data);

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
        void baca_read_cb(const mrs_msgs::BacaProtocol serial_msg);

        /**
         * @brief Peridocally send beacon frame
         * 
         */
        void beacon_timer_cb(const ros::TimerEvent&);
    };
}

#endif /* _UWB_RANGE_NODE_H_ */