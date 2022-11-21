#include <ros/ros.h>
#include "planner.h"
#include "trajectory_generator.h"
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
    // std::cout << "route:"<< std::endl<< route << std::endl;
    
    n.getParam("ts", time_list);    //提取时间参数
    time.resize(time_list.size());      //不resize 会报错
    for ( int i = 0 ; i < time_list.size() ; i++ )
    {
        XmlRpc::XmlRpcValue value = time_list[i];
        time( i ) = double(value);
        // ROS_INFO("time: %.2f",time( i ));
    }
    // std::cout << "times:"<< std::endl<< time << std::endl;
    
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
    // Eigen::MatrixXd polycoeff = getcoeff();    //获取参数矩阵     //直接使用获取参数矩阵函数获取，不采用形参
    Eigen::Vector3d ret;
    poly_coeff_num= 2 * mode;
    // std::cout << "poly_coeff_num:" << poly_coeff_num << std::endl;             //正确

    for (int dim = 0; dim < 3; dim++) 
    {
        //把参数矩阵打印出来  为空？   //直接使用获取参数矩阵函数获取，不采用形参
        //  std::cout << "polyCoeff:" << polyCoeff << std::endl;  

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
        ret(dim) = temp_pose;
    }

    std::cout << "pose:" << ret << std::endl;       //直接使用获取参数矩阵函数获取，不采用形参，获取位置成功
    return ret;
}

nav_msgs::Path planner::trajectory_path(void)
{
    nav_msgs::Path trajectory;
    geometry_msgs::PoseStamped pt;
    Vector3d pos;             //用于获取getPosPoly返回的向量，转化为pose信息
    trajectory.header.frame_id = "/map";
    trajectory.header.stamp = ros::Time();
    pt.header.frame_id="/map";
    pt.header.stamp = ros::Time();

    ros::Duration gen_time = ros::Duration();
    for (int i = 0; i < time.size(); i++) 
    {
        cout << "time:" << time(i) << endl;
        for (double t = 0.0; t < time(i); t += 0.01) 
        {
            pos = getPosPoly(poly_coeff, i, t);
            pt.pose.position.x = pos(0);
            pt.pose.position.y= pos(1);
            pt.pose.position.z = pos(2);
            trajectory.poses.push_back(pt);
        }
    }
    cout << "trajectory:" << trajectory << endl;
    return trajectory;              //  返回产生的 path 信息
}

