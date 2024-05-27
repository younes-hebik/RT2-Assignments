/**
 * \file NodeB.cpp
 * \brief Implementation of a ROS node to track the last goal target set by the user and provide it via a service.
 * \author Younes Hebik
 * \date 03/03/2024
 *
 * This node subscribes to the "reaching_goal/goal" topic to track the last target coordinates set by the user.
 * It also provides a service to return the last target coordinates when requested.
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment_2_2023/PlanningAction.h>
#include <assignment_2_2023/Custom.h>
#include <assignment_2_2023/Csrv.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <iostream>

float target_x; ///< Value of the goal's x coordinate.
float target_y; ///< Value of the goal's y coordinate.

/**
 * \brief Callback function for receiving goal messages from the action server.
 * \param goal Pointer to the goal message containing the target position.
 *
 * This function is called whenever a new goal is set. It updates the target_x and target_y
 * variables with the coordinates of the last goal set by the user.
 *
 * \note Subscribed to the "reaching_goal/goal" topic.
 */
void chatterCallback3(const assignment_2_2023::PlanningActionGoal::ConstPtr& goal)
{
    target_x = goal->goal.target_pose.pose.position.x;
    target_y = goal->goal.target_pose.pose.position.y;
}

/**
 * \brief Service callback function to provide the last target coordinates.
 * \param req Request part of the service, not used in this case.
 * \param res Response part of the service, containing the last goal's x and y coordinates.
 * \return Always returns true.
 *
 * This function is called when the "/Last_Target" service is requested. It sets the response
 * fields goal_x and goal_y to the last recorded target coordinates.
 *
 * \note Advertised as the "/Last_Target" service.
 */
bool my_random(assignment_2_2023::Csrv::Request &req, assignment_2_2023::Csrv::Response &res)
{
    res.goal_x = target_x;
    res.goal_y = target_y;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "last_target");
    ros::NodeHandle nh;

    
    ros::ServiceServer service = nh.advertiseService("/Last_Target", my_random);///< Advertise the service to provide the last target coordinates.

    
    ros::Subscriber sub = nh.subscribe("reaching_goal/goal", 10, chatterCallback3); ///< Subscribe to the "reaching_goal/goal" topic to track the last goal target set by the user.

    ros::spin();

    return 0;
}

