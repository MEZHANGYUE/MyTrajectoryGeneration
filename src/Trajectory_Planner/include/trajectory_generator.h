#ifndef _TRAJECTORY_GENERATOR_H_
#define _TRAJECTORY_GENERATOR_H_

#include <Eigen/Eigen>
#include <vector>
class TrajectoryGeneratorTool 
{
private:
    int Factorial(int x);   //求阶乘

public:
    TrajectoryGeneratorTool() = default;

    ~TrajectoryGeneratorTool() = default;

    /*求解闭式解，系数矩阵*/
    Eigen::MatrixXd SolveQPClosedForm(
            int order,
            const Eigen::MatrixXd &Path,
            const Eigen::MatrixXd &Vel,
            const Eigen::MatrixXd &Acc,
            const Eigen::VectorXd &Time);

};

#endif