// #ifndef ROS_NODE.H
// #define ROS_NODE.H
#include <Eigen/Core>
#include <Eigen/Geometry>
 #include "trajectory_generator.h"

class  TrajectoryGeneratorTool;
class planner
{
    private:
        int  dot_num ;      //保存约束点个数       
        Eigen::MatrixXd route;          //矩阵保存约束点
        Eigen::VectorXd time ;               //向量保存轨迹时间
        Eigen::MatrixXd poly_coeff;     //   求系数矩阵

    public:
        planner()                               //构造函数；
        {
            planner::getparam();            
        }     
        friend class TrajectoryGeneratorTool;   //  友元类
        void getparam(void);    //获取参数
        Eigen::MatrixXd getcoeff( void);
};

// #endif