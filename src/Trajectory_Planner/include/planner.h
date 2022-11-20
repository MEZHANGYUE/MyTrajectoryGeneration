// #ifndef ROS_NODE.H
// #define ROS_NODE.H
#include <Eigen/Core>
#include <Eigen/Geometry>
 #include "trajectory_generator.h"
using namespace std;
using namespace Eigen;
#define snap 4;
#define jerk 3;


class  TrajectoryGeneratorTool;
class planner
{
    private:
        int  dot_num ;      //保存约束点个数       
        Eigen::MatrixXd route;          //矩阵保存约束点
        Eigen::VectorXd time ;               //向量保存轨迹时间
        int poly_coeff_num;
        int mode = snap;
    public:
        Eigen::MatrixXd poly_coeff;     //   系数矩阵
        planner()                               //构造函数；
        {
            // planner::getcoeff();   
            planner::getPosPoly(planner::poly_coeff , 2 , 0.05);
            planner::trajectory_path();
        }     
        
        // friend class TrajectoryGeneratorTool;   //  友元类
        void getparam(void);    //获取参数
        Eigen::MatrixXd getcoeff( void);
        Eigen::Vector3d getPosPoly(Eigen::MatrixXd polyCoeff, int k, double t) ;
        void trajectory_path(void);
};

// #endif