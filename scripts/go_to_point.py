#! /usr/bin/env python

## @package rt2_assignment1
# \file go_to_point.py
# \brief Node implements a service to drive a robot
#        towards a point in the environment. 
# \author Jerin Joy, Carmine Recchiuto
# \date 2022-05-30
#
# \details
#
# Publishes to:<BR>
#   /cmd_vel 
#
# ServiceServer:<BR>
#   /set_vel 
#
# ActionServer:<BR>
#   /go_to_point 
#
# Description:
#
# Using an action server, this node controls
# a non-holonomic robot to produce a go_to_point
# behaviour.
# An FSM is used to model the behavior whenever
# a new goal pose is received.
##


import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
import math
import actionlib
import rt2_assignment1.msg

# action server
act_s = None

# robot state variables
position_ = Point()
yaw_ = 0
position_ = 0
state_ = 0

# publisher
pub_ = None

# parameters for control
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6

##
# \brief Odometry Callback
# \param msg: Odometry message
# 
# Retrieve (x,y & theta) from the odom message.
##

def clbk_odom(msg):
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

##
# \brief Function to specify the state_ value
# \param state: new state
#
# Updates the current global state
##

def change_state(state):
    global state_
    state_ = state
    print('State changed to [%s]' % state_)

##
# \brief Function to normalize an angle
#
# \param angle: angle to be normalized
#
# \return angle: normalized angle   
##    


def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

##
# \brief Orient the robot in a desired way
#
# \param des_yaw:  desired yaw
##    
  

def fix_yaw(des_pos):
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(1)

##
# \brief Move the robot straight to the target
#
# \param des_pos: desired (x, y) position
#
#  Set the linear and angular speed depending
#  on the distance to the goal.
##          

def go_straight_ahead(des_pos):
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw
        pub_.publish(twist_msg)
    else:  # state change conditions
        #print ('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(0)

##
# \brief Turns the robot to reach
#        the final desired yaw
#
# \param des_yaw:  desired final yaw
##      

def fix_final_yaw(des_yaw):
    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        change_state(3)

##
# \brief Stop the robot
#
# \param None
#
# Set the robot velocities to 0.
##  

def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)

##
# \brief Set the appropriate behaviour depending
#        on the current robot state, in order
#        to reach the goal.
# \param req: (x,y,theta) goal pose 
#
# The state machine keeps running until
# the goal is reached or the action is
# preempted (the goal gets cancelled). 
##

def go_to_point(req):

    global act_s

    desired_position = Point()
    desired_position.x = req.x
    desired_position.y = req.y
    des_yaw = req.theta
    change_state(0)

    feedback = rt2_assignment1.msg.MoveFeedback()
    result = rt2_assignment1.msg.MoveResult()

    while True:
        if act_s.is_preempt_requested():
            rospy.loginfo('Goal was preempted')
            act_s.set_preempted()
            success = False
            done()
            break
        elif state_ == 0:
            fix_yaw(desired_position)
        elif state_ == 1:
            go_straight_ahead(desired_position)
        elif state_ == 2:
            fix_final_yaw(des_yaw)
        elif state_ == 3:
            done()
            success = True
            break
    if success:
        act_s.set_succeeded(result)
    return True

##
# \brief Main function to manage 
#        the robot behaviour
#
# handles:
# - the initialization of the "go_to_point" node
# - the publisher for the "\cmd_vel" topic
# - the subscriber to the "\odom" topic
# - the action server "\go_to_point"
##

def main():
    global pub_
    global act_s
    rospy.init_node('go_to_point')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    act_s = actionlib.SimpleActionServer(
        '/go_to_point', rt2_assignment1.msg.MoveAction, go_to_point, auto_start=False)
    act_s.start()
    rospy.spin()


if __name__ == '__main__':
    main()
