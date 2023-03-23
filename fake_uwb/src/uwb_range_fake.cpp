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

#include <random>
#include <cstdint>

#include<sstream>
#include<iostream>

#include <math.h>
#include <cmath>

geometry_msgs::Point pose1;
geometry_msgs::Point pose2;

bool first_pos = false;
bool second_pos = false;

void cb_1(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    first_pos = true;
    pose1 = msg->pose.pose.position;
    return;
}

void cb_2(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    second_pos = true;
    pose2 = msg->pose.pose.position;
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uwb_range_fake");
    ros::NodeHandle nh("~");

    ROS_INFO("[UWB_RANGER]: Node set");

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);

    std::normal_distribution<double> normal_dist(0, 0.05);

    ros::Subscriber sub1 = nh.subscribe("/uav1/ground_truth_pose", 100, cb_1);
    ros::Subscriber sub2 = nh.subscribe("/uav2/ground_truth_pose", 100, cb_2);

    ros::Publisher dist_pub = nh.advertise<mrs_msgs::RangeWithCovarianceArrayStamped>("distance", 1000);

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

        range.id = 0;
        range.variance = 0.22;

        range.range.field_of_view = 2*M_PI;
        range.range.radiation_type = 3;
        range.range.min_range = 0;
        range.range.max_range = 100;

        range.range.header.stamp = ros::Time::now();
        range.range.range = dist + normal_dist(generator);

        ROS_INFO("[UWB_RANGER]: Publishing range: %.2f m", range.range.range);

        out_msg.ranges.push_back(range);
        out_msg.header.stamp = ros::Time::now();

        dist_pub.publish(out_msg);
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}