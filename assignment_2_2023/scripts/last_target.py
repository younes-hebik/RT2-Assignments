#!/usr/bin/env python

import rospy
from assignment_2_2023.msg import PlanningActionGoal
from assignment_2_2023.srv import Csrv, CsrvResponse

# Global variables to store the last target goal coordinates
target_x = 0.0
target_y = 0.0

# Callback function to update the last goal target coordinates
def goal_callback(goal):
    global target_x, target_y
    target_x = goal.goal.target_pose.pose.position.x
    target_y = goal.goal.target_pose.pose.position.y

# Service handler function to return the last goal target coordinates
def handle_last_target(req):
    res = CsrvResponse()
    res.goal_x = target_x
    res.goal_y = target_y
    return res

def main():
    rospy.init_node('last_target')
  

    # Advertise the service to provide the last target coordinates
    service = rospy.Service('/Last_Target', Csrv, handle_last_target)
    
    # Subscribe to the "reaching_goal/goal" topic to get the latest goal target
    rospy.Subscriber("reaching_goal/goal", PlanningActionGoal, goal_callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()

