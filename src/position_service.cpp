/**
 * @file position_service.cpp
 * @author Jerin Joy
 * @brief Node responsible for generating a random position
 * @date 2022-05-30
 * 
 * @details
 * 
 * ServiceServer:<BR>
 *   /position_server (rt2_assignment1::RandomPosition)
 *
 * Description:
 *
 * This node replies to a request for a random
 * pose (x, y, theta) with a random pose bounded
 * by the limits passed in the request.  
 *         
 * @copyright Copyright (c) 2022
 * 
 */

#include "ros/ros.h"
#include "rt2_assignment1/RandomPosition.h"

/**
 * @brief Generate a random number
 * 
 * @param M lower bound 
 * @param N upper bound
 * @return random number between M and N. 
 */
double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }

/**
 * @brief Service callback that generates a random pose.
 * 
 * @param req Service request
 * @param res Service response
 * @return true or false 
 */
bool myrandom (rt2_assignment1::RandomPosition::Request &req, rt2_assignment1::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "random_position_server");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/position_server", myrandom);
   ros::spin();

   return 0;
}
