/**
 * @file state_machine.cpp
 * @author Jerin Joy
 * @brief Node that issues new goals for the robot
 * @date 2022-05-30
 * 
 * @details
 *
 * ServiceServer:<BR>
 *   /user_interface 
 *
 * ServiceClient:<BR>
 *   /position_server 
 *
 * ActionClient:<BR>
 *   /go_to_point
 *
 * Description:
 *
 * This node communicates with the action server for
 * go_to_point, issuing new goals whenever a 'start'
 * request is received from the user_interface.
 * At the same time it can stop a running goal
 * if a request for 'stop' is received from the
 * user_interface.
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/Position.h"
#include "rt2_assignment1/RandomPosition.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rt2_assignment1/MoveAction.h>
#include "geometry_msgs/Twist.h"

bool start = false;
bool stop = false;

/**
 * @brief Service callback that sets the robot state
 * 
 * @param req Service request
 * @param res Service response
 * @return true
 *  
 */
bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res)
{
    if (req.command == "start")
    {
        start = true;
        stop = false;
    }   
    else
    {
        start = false;
        stop = true;
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_machine");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("/user_interface", user_interface);
    ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    actionlib::SimpleActionClient<rt2_assignment1::MoveAction> ac("/go_to_point", true);

    rt2_assignment1::RandomPosition rp;
    rt2_assignment1::MoveGoal goal;

    rp.request.x_max = 5.0;
    rp.request.x_min = -5.0;
    rp.request.y_max = 5.0;
    rp.request.y_min = -5.0;
    geometry_msgs::Twist twist_msg;

    while (ros::ok())
    {
        ros::spinOnce();
        if (start)
        {
            client_rp.call(rp);
            goal.x = rp.response.x;
            goal.y = rp.response.y;
            goal.theta = rp.response.theta;
            std::cout << "\nGoing to the position: x= " << goal.x << " y= " << goal.y << " theta = " << goal.theta << std::endl;
            ROS_INFO("Sending goal");
            ac.sendGoal(goal);

            while (true)
            {
                ros::spinOnce();
                if (stop == false)
                {

                    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    {
                        ROS_INFO("Hooray, target reached!");
                        break;
                    }
                }
                if (stop == true)
                {

                    ac.cancelGoal();

                    // twist_msg.linear.x = 0;
                    // twist_msg.linear.y = 0;
                    // twist_msg.angular.z = 0;
                    // pub.publish(twist_msg);
                    break;
                }
            }

            std::cout << "Position reached" << std::endl;
            // break;        //Uncomment this line to stop the robot at the initial goal.
        }
    }
    return 0;
}
