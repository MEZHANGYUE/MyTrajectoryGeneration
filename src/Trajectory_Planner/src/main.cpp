#include <ros/ros.h>
 #include "planner.h"
#include <thread>


int main(int argc , char ** argv)
{
    ros::init(argc,argv,"ros_node");   //ROS初始化
    planner planner;
    // planner.getPosPoly(planner.poly_coeff, 2, 0.05);

    // std::thread thrd_1(&planner::getparam, &node);
    // std::thread thrd_2(&planner::getpath, &node);
    // thrd_1.join();
    // thrd_2.join();

}