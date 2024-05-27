#!/usr/bin/env python

import rospy
import actionlib
from assignment_2_2023.msg import PlanningAction, PlanningGoal, PlanningActionFeedback, Custom
from nav_msgs.msg import Odometry

# Global variables for robot position and status
latestX = 0.0
latestY = 0.0
state = ""
od_x = 0.0
od_y = 0.0
od_vx = 0.0
od_vy = 0.0

# Publisher for custom message
pub = None

# Callback for the "reaching_goal/feedback" topic
def feedback_callback(feed):
    global latestX, latestY, state
    latestX = feed.feedback.actual_pose.position.x
    latestY = feed.feedback.actual_pose.position.y
    state = feed.feedback.stat

# Callback for the "odom" topic
def odom_callback(od):
    global od_x, od_y, od_vx, od_vy
    od_x = od.pose.pose.position.x
    od_y = od.pose.pose.position.y
    od_vx = od.twist.twist.linear.x
    od_vy = od.twist.twist.linear.y

    # Publish custom message
    cmsg = Custom()
    cmsg.posx = od_x
    cmsg.posy = od_y
    cmsg.velx = od_vx
    cmsg.vely = od_vy
    pub.publish(cmsg)
    
    


def main():
    global pub

    rospy.init_node('actcl')

    # Action client for "reaching_goal"
    ac = actionlib.SimpleActionClient('reaching_goal', PlanningAction)

    rospy.loginfo("Waiting for action server to start.")
    ac.wait_for_server()
    rospy.loginfo("Action server started, sending goal.")

    # Subscribers and publisher
    rospy.Subscriber("reaching_goal/feedback", PlanningActionFeedback, feedback_callback)
    rospy.Subscriber("odom", Odometry, odom_callback)
    pub = rospy.Publisher("custom_pos_vel", Custom, queue_size=1)

    while not rospy.is_shutdown():
        print("Enter your choice \n1- for setting target \n2- for cancel the process \n3- for getting the coordinates and the state of the robot): ")
        choice = int(input())

        if choice == 1:
            print("You chose Option 1.")
            x = float(input("Enter the value of x: "))
            y = float(input("Enter the value of y: "))

            goal = PlanningGoal()
            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y

            ac.send_goal(goal)

        elif choice == 2:
            print("You chose Option 2.")
            ac.cancel_goal()
            

        elif choice == 3:
            rospy.loginfo(f"the position of X is: [{latestX}]")
            rospy.loginfo(f"the position of Y is: [{latestY}]")
            rospy.loginfo(f"the status: {state}")
            print(state)

        else:
            print("You chose the wrong answer.")

        

if __name__ == '__main__':
    main()

