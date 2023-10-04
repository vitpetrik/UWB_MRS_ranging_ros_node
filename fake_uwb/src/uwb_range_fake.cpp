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
#include <mrs_msgs/RangeWithCovarianceArrayStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/NavSatFix.h>
#include <mrs_msgs/NavSatFixArrayStamped.h>
#include <nav_msgs/Odometry.h>

#include <mrs_lib/param_loader.h>

#include <random>
#include <cstdint>

#include<sstream>
#include<iostream>

#include <math.h>
#include <cmath>

geometry_msgs::Point pose1;
geometry_msgs::Point pose2;

ros::Publisher gps_pub;

bool first_pos = false;
bool second_pos = false;

int output_id = 0;
double altitude = nan("");

void cb_1(const nav_msgs::Odometry::ConstPtr& msg)
{
    first_pos = true;
    pose1 = msg->pose.pose.position;
    return;
}

void cb_2(const nav_msgs::Odometry::ConstPtr& msg)
{
    second_pos = true;
    pose2 = msg->pose.pose.position;
    return;
}

void gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    mrs_msgs::NavSatFixArrayStamped out_msg;
    mrs_msgs::NavSatFixIdentified gps;

    gps.id = output_id;
    gps.gps = *msg;

    gps.gps.altitude = altitude;
    
    out_msg.nav_sat_fixes.push_back(gps);
    out_msg.header.stamp = msg->header.stamp;
    out_msg.header.frame_id = "gps";

    gps_pub.publish(out_msg);

    return;
}

void odometry_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    altitude = msg->pose.pose.position.z;
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uwb_range_fake");
    ros::NodeHandle nh("~");

    mrs_lib::ParamLoader param_loader(nh, "Uwb fake");

    ROS_INFO("[UWB_RANGER]: Node set");

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);

    std::string uav_name;
    std::string target_uav;
    std::string output_frame;

    param_loader.loadParam("uav", uav_name);
    param_loader.loadParam("target_uav", target_uav);
    param_loader.loadParam("output_id", output_id);
    param_loader.loadParam("output_frame", output_frame);

    std::normal_distribution<double> normal_dist(0, 0.05);

    ros::Subscriber sub1 = nh.subscribe("gt_observer", 10, cb_1);
    ros::Subscriber sub2 = nh.subscribe("gt_target", 10, cb_2);

    ros::Subscriber sub_gps = nh.subscribe("gps_target", 10, gps_cb);
    ros::Subscriber sub_odometry = nh.subscribe("odom_target", 10, odometry_cb);

    ros::Publisher dist_pub = nh.advertise<mrs_msgs::RangeWithCovarianceArrayStamped>("range_out", 1);
    gps_pub = nh.advertise<mrs_msgs::NavSatFixArrayStamped>("gps_out", 1);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        if (not first_pos or not second_pos)
        {
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }

        double dist = sqrt(pow((pose1.x - pose2.x), 2) + 
                            pow((pose1.y - pose2.y), 2) + 
                            pow((pose1.z - pose2.z), 2));

        mrs_msgs::RangeWithCovarianceArrayStamped out_msg;
        mrs_msgs::RangeWithCovarianceIdentified range;

        range.id = output_id;
        range.variance = 0.0025;

        range.range.field_of_view = 2*M_PI;
        range.range.radiation_type = 3;
        range.range.min_range = 0;
        range.range.max_range = 100;

        range.range.header.stamp = ros::Time::now();
        range.range.range = dist + normal_dist(generator);

        ROS_INFO("[UWB_RANGER]: Publishing range: %.2f m", range.range.range);

        out_msg.ranges.push_back(range);
        out_msg.header.stamp = ros::Time::now();
        out_msg.header.frame_id = output_frame;

        dist_pub.publish(out_msg);
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}