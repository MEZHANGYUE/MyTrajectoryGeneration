#include <ros/ros.h>
 #include "planner.h"
#include <thread>


int main(int argc , char ** argv)
{
    ros::init(argc,argv,"ros_node");   //ROS初始化
    planner node;
    //node.getparam();
    // std::thread thrd_1(&planner::getparam, &node);
    // std::thread thrd_2(&planner::getpath, &node);
    // thrd_1.join();
    // thrd_2.join();

}