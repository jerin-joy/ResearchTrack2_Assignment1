#! /usr/bin/env python

"""
.. module:: go_to_point.py
    :platform: Unix
    :synopsis: Node implements a service to drive a robot
               towards a point in the environment.
.. moduleauthor: Jerin Joy, Carmine Recchiuto
      

Publishes to:
    /cmd_vel 

ActionServer:
   /go_to_point 

Description:

Using an action server, this node controls
a non-holonomic robot to produce a go_to_point
behaviour.
An FSM is used to model the behavior whenever
a new goal pose is received.

"""


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


def clbk_odom(msg):

    """
    Odometry Callback

    Retrieve (x,y & theta) from the odom message.
   
    Args:
     msg: Odometry message 

    """

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


def change_state(state):
    """
    Function to specify the state_ value and updates the current global state.

    Args:
     state: new state
    
    Updates the current global state

    """

    global state_
    state_ = state
    print('State changed to [%s]' % state_)
 


def normalize_angle(angle):

    """
    Function to normalize an angle
    
    Args:
     angle: angle to be normalized
    
    Returns:
     angle: normalized angle  

    """
       
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle
  

def fix_yaw(des_pos):
    """
    Orient the robot in a desired way
    
    Args:
     des_yaw:  desired yaw

    """ 

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

        

def go_straight_ahead(des_pos):
    """
    Move the robot straight to the target

    Set the linear and angular speed depending
    on the distance to the goal.
    
    Args:
     des_pos: desired (x, y) position

    """  

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
  

def fix_final_yaw(des_yaw):
    """
    Turns the robot to reach
    the final desired yaw
    
    Args:
     des_yaw:  desired final yaw

    """

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


def done():
    """
    Stop the robot

    Set the robot velocities to 0.
    
    Args:
     None

    """  
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)


def go_to_point(req):
    """
    Set the appropriate behaviour depending
    on the current robot state, in order
    to reach the goal.

    The state machine keeps running until
    the goal is reached or the action is
    preempted (the goal gets cancelled). 

    Args:
     req: (x,y,theta) goal pose 

    """

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



def main():
    """
    Main function to manage 
    the robot behaviour

    handles:
    - the initialization of the "go_to_point" node
    - the publisher for the "\cmd_vel" topic
    - the subscriber to the "\odom" topic
    - the action server "\go_to_point"
    
    """
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
