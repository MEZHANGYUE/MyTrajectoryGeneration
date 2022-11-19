#include <ros/ros.h>
#include "planner.h"

using namespace std;

/*
*获取配置文件参数
*/
void planner::getparam(void)
{
    ros::NodeHandle n ;
    dot_num = 0;

    XmlRpc::XmlRpcValue param_list;    //严格限定数据类型
    XmlRpc::XmlRpcValue time_list;
    n.getParam("pose", param_list);   //提取约束点参数
    n.getParam("ts", time_list);    //提取时间参数

    dot_num = param_list.size() / 3;
    route.resize (3 , dot_num);                           //不resize 会报错
    for (int i = 0 ; i < param_list.size() ; i++ )
    {
        XmlRpc::XmlRpcValue value = param_list[i];
        route(i / dot_num , i % dot_num) = double(value);
        // ROS_INFO("DOT: %.2f",route(i / dot_num , i % dot_num));
    }
    std::cout << "route:"<< route << std::endl;
    
    n.getParam("ts", time_list);    //提取时间参数
    time.resize(time_list.size());      //不resize 会报错
    for ( int i = 0 ; i < time_list.size() ; i++ )
    {
        XmlRpc::XmlRpcValue value = time_list[i];
        time( i ) = double(value);
        // ROS_INFO("time: %.2f",time( i ));
    }
    std::cout << "times:"<< time << std::endl;
    
}
