#include <ros/ros.h>
#include "planner.h"
#include <vector>
#include <Eigen/Eigen>
#include "trajectory_generator.h"
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
    route.resize (dot_num, 3  );                           //不resize 会报错
    for (int i = 0 ; i < param_list.size() ; i++ )
    {
        XmlRpc::XmlRpcValue value = param_list[i];
        route(  i % dot_num , i / dot_num) = double(value);
    }
    std::cout << "route:"<< std::endl<< route << std::endl;
    
    n.getParam("ts", time_list);    //提取时间参数
    time.resize(time_list.size());      //不resize 会报错
    for ( int i = 0 ; i < time_list.size() ; i++ )
    {
        XmlRpc::XmlRpcValue value = time_list[i];
        time( i ) = double(value);
        // ROS_INFO("time: %.2f",time( i ));
    }
    std::cout << "times:"<< std::endl<< time << std::endl;
    
}

/*
*获取参数矩阵
*/
Eigen::MatrixXd planner::getcoeff(void)
{
    Eigen::MatrixXd poly_coeff;
    Eigen::MatrixXd vel = Eigen::MatrixXd::Zero(2, 3);
    Eigen::MatrixXd acc = Eigen::MatrixXd::Zero(2, 3);
    TrajectoryGeneratorTool TrajectoryGeneratorTool;
    poly_coeff = TrajectoryGeneratorTool.SolveQPClosedForm(4, route, vel, acc, time);
    // std::cout << "poly_coeff:" << std::endl << poly_coeff << std::endl;
    return poly_coeff;
}