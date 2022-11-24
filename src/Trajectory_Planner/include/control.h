#ifndef CONTROL.H
#define CONTROL.H
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h> 
#include <mavros_msgs/CommandBool.h> 
#include <mavros_msgs/SetMode.h>   
#include <mavros_msgs/State.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include "geometry_msgs/TwistStamped.h"
struct XYZ
{
    float x;
    float y;
    float z;
};
class control
{
    private:
        ros::NodeHandle nh;
        ros::ServiceClient arming_client;     //解锁服务
        ros::ServiceClient set_mode_client;    //模式设置   offboard
        ros::Publisher local_pos_pub;  ///发布无人机位置控制
        ros::Publisher local_vel_pub;  ///发布无人机速度控制
        ros::Subscriber state_sub;
        ros::Subscriber pose_sub;
        ros::Subscriber path_sub;      // 订阅生成的优化轨迹
        ros::Publisher realpath_pub;   // 发布实际生成的轨迹
        geometry_msgs::TwistStamped vel;
        XYZ curr_error;
        XYZ last_error;
        float kp , ki , kd;
    public:
        control()                               //构造函数；
        {
            // control::px4_init();
            control::px4_control();
            // std::cout <<"plan_tra:" << control::plan_tra << std::endl;
        }     
        mavros_msgs::SetMode offb_set_mode;
        mavros_msgs::CommandBool arm_cmd;
        int mode_set (void);
        void px4_init(void);
        void planpath_sub(void);   //订阅轨迹
        geometry_msgs::TwistStamped PID_vel(geometry_msgs::PoseStamped cuur_pose , geometry_msgs::PoseStamped targ_pose);  //计算PID
        void px4_follow(void);
        void px4_control(void);
        //两个回调函数
        // void trajectory_cb(const nav_msgs::Path::ConstPtr &wp);  //订阅轨迹回调
        // void state_cb(const mavros_msgs::State::ConstPtr& msg);  //订阅无人机状态
        
};

#endif