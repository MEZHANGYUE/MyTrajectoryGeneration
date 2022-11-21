 #include "planner.h"
#include <thread>


int main(int argc , char ** argv)
{
    ros::init(argc,argv,"ros_node");   //ROS初始化
    ros::NodeHandle nh ;
    planner planner;
    nav_msgs::Path tra;
    ros::Publisher tra_generation_pub = nh.advertise<nav_msgs::Path>("/tra_generation", 10); 
    // tra = planner.trajectory_path();
    // while (!ros::ok());
    // tra_generation_pub.publish(tra);
    
    // planner.tra_publish();
    // ros::Rate rate(1);  //设置发布频率
    // while (ros::ok()) 
    // {
    //     tra_generation_pub.publish(tra);
    //     // planner.tra_publish();
    //     rate.sleep();
    //     ros::spinOnce();
    // }
    return 0;
    // std::thread thrd_1(&planner::getparam, &node);
    // std::thread thrd_2(&planner::getpath, &node);
    // thrd_1.join();
    // thrd_2.join();

}