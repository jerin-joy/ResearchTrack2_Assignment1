## @package rt2_assignment1
# \file user_interface.py
# \brief Displays the user interface to control the robot
# \author Jerin Joy, Carmine Recchiuto
# \date 2022-05-30
#
# \details
#
# \Client: <BR>
#          \user_interface
#
# Description:
#
# This node define the command line user interface
# for the control of the robot
##

import rospy
import time
from rt2_assignment1.srv import Command


def main():
    rospy.init_node('user_interface')
    ui_client = rospy.ServiceProxy('/user_interface', Command)
    time.sleep(10)
    rate = rospy.Rate(20)
    x = int(input("\nPress 1 to start the robot "))

    while not rospy.is_shutdown():
        if (x == 1):
            ui_client("start")
            x = int(input("\nPress 0 to stop the robot "))
        elif (x == 0):
            print("\nCancelling the goal and stopping the robot.")
            ui_client("stop")
            x = int(input("\nPress 1 to start the robot "))
            # break
        else:
            x = int(input("\nInvalid input. Press either 0 or 1."))


if __name__ == '__main__':
    main()
