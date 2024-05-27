/**
 * \file NodeC.cpp
 * \brief Implementation of a ROS node that serves as a server to provide the average speed and the distance of the robot from the goal.
 * \author Younes Hebik
 * \date 03/03/2024
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment_2_2023/PlanningAction.h>
#include <assignment_2_2023/Custom.h>
#include <assignment_2_2023/Csrv.h>
#include <assignment_2_2023/Avrg.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <iostream>

using namespace std;

float dis_x; ///< Position of the robot on the x-axis.
float dis_y; ///< Position of the robot on the y-axis.
float speed_x; ///< Velocity of the robot on the x-axis.
float speed_y; ///< Velocity of the robot on the y-axis.
float final_x; ///< Goal position on the x-axis.
float final_y; ///< Goal position on the y-axis.
float dx;
float dy;
float d; ///< Distance left to reach the goal.
float spd = 0.0;
int i;
float spdf; ///< Average speed.

/**
 * \brief Callback function to get the instantaneous position coordinates and velocity from the odom topic.
 * \param od1 Odometry message containing the robot's position and velocity.
 *
 * This function updates the global variables dis_x, dis_y, speed_x, and speed_y with the robot's current position and velocity.
 * 
 * \note Subscribed to the "odom" topic.
 */
void chatterCallback4(const nav_msgs::Odometry::ConstPtr& od1)
{
    dis_x = od1->pose.pose.position.x;
    dis_y = od1->pose.pose.position.y;
    speed_x = od1->twist.twist.linear.x;
    speed_y = od1->twist.twist.linear.y;
}

/**
 * \brief Callback function to get the final goal position.
 * \param goal1 PlanningActionGoal message containing the target position.
 *
 * This function updates the global variables final_x and final_y with the goal's position.
 * 
 * \note Subscribed to the "reaching_goal/goal" topic.
 */
void chatterCallback5(const assignment_2_2023::PlanningActionGoal::ConstPtr& goal1)
{
    final_x = goal1->goal.target_pose.pose.position.x;
    final_y = goal1->goal.target_pose.pose.position.y;
}

/**
 * \brief Service callback function to provide the average speed and distance from the goal.
 * \param req Request part of the service (not used in this case).
 * \param res Response part of the service, containing the average speed and distance to the goal.
 * \return Always returns true.
 *
 * This function sets the response fields dis and speed with the calculated distance and average speed.
 * 
 * \note Advertised as the "/speed_distance" service.
 */
bool my_random(assignment_2_2023::Avrg::Request &req, assignment_2_2023::Avrg::Response &res)
{
    res.dis = d;
    res.speed = spdf;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "speed_node");
    ros::NodeHandle nh;

    ///< Initialize a subscriber on the odom topic.
    ros::Subscriber sub = nh.subscribe("odom", 10, chatterCallback4);

    ///< Initialize a subscriber on the reaching_goal/goal topic.
    ros::Subscriber sub2 = nh.subscribe("reaching_goal/goal", 10, chatterCallback5);

    ///< Initialize a server to provide the average speed and the distance of the robot from the goal. The service is called "/speed_distance".
    ros::ServiceServer service = nh.advertiseService("/speed_distance", my_random);

    while (ros::ok()) {
        // Collect 50 samples of the robot's speed and calculate the average.
        for (i = 0; i < 50; i++) {
            spd = spd + sqrt(speed_x * speed_x + speed_y * speed_y);
            ros::spinOnce();
        }
        spdf = spd / i;
        i = 0;
        spd = 0;

        // Calculate the distance difference between the goal and the robot's position on the x and y axes.
        dx = final_x - dis_x;
        dy = final_y - dis_y;
        d = sqrt((dx * dx) + (dy * dy));

        ros::spinOnce();
    }

    return 0;
}

