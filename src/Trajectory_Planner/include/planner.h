// #ifndef ROS_NODE.H
// #define ROS_NODE.H
#include <Eigen/Core>
#include <Eigen/Geometry>
//  #include "minimum_snap.h"
class planner
{
    private:
        int  dot_num ;      //保存约束点个数
        Eigen::MatrixXd route;          //矩阵保存约束点
        Eigen::VectorXd time ;                             //向量保存轨迹时间       

        // MinimumSnap mini_snap;       //初始化一个MinimumSnap类



    public:
        planner()                               //构造函数；
        {
            planner::getparam();

        }     
        
        void getparam(void);    //获取参数
        void getpath(void);   //获取路径
};

// #endif