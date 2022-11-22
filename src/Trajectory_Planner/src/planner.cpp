#include <ros/ros.h>
#include "planner.h"
#include "trajectory_generator.h"
/*
*获取配置文件参数
*/
void planner::getparam(void)
{
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
    // std::cout << "route:"<< std::endl<< route << std::endl;
    
    n.getParam("ts", time_list);    //提取时间参数
    time.resize(time_list.size());      //不resize 会报错
    for ( int i = 0 ; i < time_list.size() ; i++ )
    {
        XmlRpc::XmlRpcValue value = time_list[i];
        time( i ) = double(value);
    }
}

/*
*获取路径参数矩阵
*/
Eigen::MatrixXd planner::getcoeff(void)
{
    Eigen::MatrixXd polycoeff;
    Eigen::MatrixXd vel = Eigen::MatrixXd::Zero(2, 3);
    Eigen::MatrixXd acc = Eigen::MatrixXd::Zero(2, 3);
    TrajectoryGeneratorTool TrajectoryGeneratorTool;
    getparam();
    polycoeff = TrajectoryGeneratorTool.SolveQPClosedForm(mode, route, vel, acc, time);
    // std::cout << "poly_coeff:" << std::endl << poly_coeff << std::endl;
    return polycoeff; 
}


/*!
 * 求解第k个轨迹段t时刻对应的位置
 * @param polyCoeff 多项式系数矩阵
 * @param k 轨迹段序号
 * @param t 时刻
 * @return [x,y,z]^T
 */
Eigen::Vector3d planner::getPosPoly(Eigen::MatrixXd polyCoeff, int k, double t) 
{
    Eigen::Vector3d pt;
    poly_coeff_num= 2 * mode;
    // std::cout << "poly_coeff_num:" << poly_coeff_num << std::endl;             //正确

    for (int dim = 0; dim < 3; dim++) 
    {

        Eigen::VectorXd coeff;
        coeff.resize(poly_coeff_num);
        
        coeff = (polyCoeff.row(k)).segment(dim * poly_coeff_num, poly_coeff_num);
        Eigen::VectorXd times = Eigen::VectorXd::Zero(poly_coeff_num);
        for (int j = 0; j < poly_coeff_num; j++)
            if (j == 0)
                times(j) = 1.0;
            else
                times(j) = pow(t, j);
        double temp_pose = 0.0;
        for (int i = 0; i < times.rows(); ++i) 
        {
            temp_pose = temp_pose + coeff(i) * times(times.rows() - i - 1);
        }
        pt(dim) = temp_pose;
    }

    // std::cout << "pose:" << pt << std::endl;       //获取位置成功
    return pt;
}

nav_msgs::Path planner::trajectory_path(void)
{
    nav_msgs::Path trajectory;
    geometry_msgs::PoseStamped pt;
    Vector3d pos;             //用于获取getPosPoly返回的向量，转化为pose信息
    trajectory.header.frame_id = "/map";
    trajectory.header.stamp = ros::Time::now();
    pt.header.frame_id="/map";
    pt.header.stamp = ros::Time::now();

    for (int i = 0; i < time.size(); i++) 
    {
        // cout << "time:" << time(i) << endl;
        for (double t = 0.0; t < time(i); t += 0.01) 
        {
            pos = getPosPoly(poly_coeff, i, t);
            pt.pose.position.x = pos(0);
            pt.pose.position.y= pos(1);
            pt.pose.position.z = pos(2);
            trajectory.poses.push_back(pt);
        }
    }
    // cout << "trajectory:" << trajectory << endl;
    return trajectory;              //  返回产生的 path 信息
}

/*
*轨迹发布
*/
void planner::tra_publish(void)
{
    ros::Publisher tra_generation_pub = n.advertise<nav_msgs::Path>("/tra_generation", 10); 

    nav_msgs::Path tra;
    tra = trajectory_path();   // 获取轨迹path

    // cout << "tra:" << tra << "tra:" << endl;
    // ros::Rate rate(1);  //设置发布频率
    // rate.sleep();
    // tra_generation_pub.publish(tra);   // 发布轨迹path
    
    ros::Rate rate(1);  //设置发布频率
    while (1) 
    {
        tra_generation_pub.publish(tra);
        rate.sleep();
        ros::spinOnce();
    }
}


