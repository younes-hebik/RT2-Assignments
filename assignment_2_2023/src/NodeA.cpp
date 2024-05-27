/**
 * \file NodeA.cpp
 * \brief Implementation of an action client in ROS for setting new targets, canceling goals, and monitoring robot position and velocity.
 * \author Younes Hebik
 * \date 03/03/2024
 *
 * This node, named "NodeA", is responsible for interacting with an action server to set new targets,
 * cancel goals, and monitor the robot's position and velocity. It subscribes to feedback and odometry
 * topics and publishes custom messages with the robot's position and velocity. <BR>
 * Subscribes To: <BR>
 *	"reaching_goal/feedback" <BR>
 *	"odom"<BR>
 * Publishes To: <BR>
 * "custom_pos_vel"
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment_2_2023/PlanningAction.h>
#include <assignment_2_2023/Custom.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <iostream>

using namespace std;

ros::Publisher pub; ///< Publisher for custom position and velocity messages.
double latestX = 0.0; ///< Latest robot position on the x-coordinate.
double latestY = 0.0; ///< Latest robot position on the y-coordinate.
string state; ///< Current state of the robot.
double od_x; ///< X position in the odometry message.
double od_y; ///< Y position in the odometry message.
double od_vx; ///< Velocity on the x-axis in the odometry message.
double od_vy; ///< Velocity on the y-axis in the odometry message.
ros::Subscriber sub; ///< Subscriber for receiving feedback from the action server.
ros::Subscriber sub2; ///< Subscriber for receiving odometry messages.

/**
 * \brief Callback function for receiving feedback from the action server.
 * \param feed Feedback containing actual pose and state.
 * This function updates the latestX, latestY, and state variables based on the received feedback.
 * It is subscribed to the "reaching_goal/feedback" topic.
 */
void chatterCallback(const assignment_2_2023::PlanningActionFeedback::ConstPtr& feed)
{
    latestX = feed->feedback.actual_pose.position.x;
    latestY = feed->feedback.actual_pose.position.y;
    state = feed->feedback.stat;
}

/**
 * \brief Callback function for receiving odometry messages.
 * \param od Odometry message containing position and velocity.
 * This function updates the od_x, od_y, od_vx, and od_vy variables based on the received odometry message.
 * Additionally, it publishes custom messages containing position and velocity information.
 * It is subscribed to the "odom" topic and publishes to the "custom_pos_vel" topic.
 */
void chatterCallback2(const nav_msgs::Odometry::ConstPtr& od)
{
    od_x = od->pose.pose.position.x;
    od_y = od->pose.pose.position.y;
    od_vx = od->twist.twist.linear.x;
    od_vy = od->twist.twist.linear.y;

    assignment_2_2023::Custom cmsg;
    cmsg.posx = od_x;
    cmsg.posy = od_y;
    cmsg.velx = od_vx;
    cmsg.vely = od_vy;
    pub.publish(cmsg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "actcl");
    ros::NodeHandle n;


    actionlib::SimpleActionClient<assignment_2_2023::PlanningAction> ac("reaching_goal", true);     ///< Intialize  the action client

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");

    assignment_2_2023::PlanningGoal goal;

    int choice, x, y;


    ros::Subscriber sub = n.subscribe("reaching_goal/feedback", 10, chatterCallback);    ///< Intialize a Subscriber  that subscribes  to feedback and odometry topics
    ros::Subscriber sub2 = n.subscribe("odom", 10, chatterCallback2);
    pub = n.advertise<assignment_2_2023::Custom>("custom_pos_vel", 1);///< Intialize a publisher  that publishes on custom_pos_vel Topic 

    while (ros::ok()) {
        cout << "Enter your choice \n1- for setting target \n2- for canceling the process \n3- for getting the coordinates and the state of the robot): ";
        cin >> choice;

        if (choice == 1) {
            cout << "You chose Option 1.\n";
            cout << "Enter the value of x: \n";
            cin >> x;
            cout << "Enter the value of y: \n";
            cin >> y;
            ac.waitForServer();
            goal.target_pose.pose.position.x = x;
            goal.target_pose.pose.position.y = y;
            ac.sendGoal(goal);
        } else if (choice == 2) {
            cout << "You chose Option 2.\n";
            ac.cancelGoal();
            break;
        } else if (choice == 3) {
            ROS_INFO("The position of X is: [%f\n]", latestX);
            ROS_INFO("The position of Y is: [%f]\n", latestY);
            ROS_INFO("The status: %s", state.c_str());
        } else {
            cout << "You chose the wrong answer";
        }

        ros::spinOnce();
    }

    return 0;
}

