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

#include <ros/ros.h>
#include <ros/package.h>
#include <mrs_msgs/BacaProtocol.h>

namespace uwb {
    class UwbRange {
        public:
            UwbRange(ros::NodeHandle nh);
            ~UwbRange();

        private:

        /** 
         * Publisher definitions
         */
            ros::Publisher baca_write;

        /** 
         * Subscribers definitions
         */
            ros::Subscriber baca_read;

        /** 
         * Callbacks
         */

        void baca_read_cb(const mrs_msgs::BacaProtocol msg);

    };
}