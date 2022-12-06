#include "control.h"
//建立一个订阅消息体类型的变量，用于存储订阅的信息
mavros_msgs::State current_state;
nav_msgs::Path plan_tra;
geometry_msgs::PoseStamped px4_pose;

//回调函数
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
    std::cout << "current_state:  " << current_state << std::endl;
}
 
 void trajectory_cb(const nav_msgs::Path::ConstPtr &wp)
{
    plan_tra = *wp;
}
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &pt)
{
    px4_pose = *pt;
    // std::cout << "px4_pose:  " << px4_pose << std::endl;
}
/*
**       初始化px4，通信连接、解锁、模式设置
*/
int control::mode_set (void)
{
    if( current_state.mode != "OFFBOARD")
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");//打开模式后打印信息
            }
        
    if( !current_state.armed)
        if( arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
            ROS_INFO("Vehicle armed");//解锁后打印信息
        }
    
    if (current_state.armed && current_state.mode == "OFFBOARD")   return 1;
    else return 0;
    
}

void control::px4_init(void)
{
    ros::Rate rate(20.0);  //设置发布速率
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    local_vel_pub = nh.advertise<geometry_msgs::TwistStamped> ("/mavros/setpoint_velocity/cmd_vel", 10); 
    realpath_pub = nh.advertise<nav_msgs::Path>("/real_trajectory", 10);
    state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, state_cb);     //订阅飞机状态
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_cb);     //订阅飞机位置pose
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");     //解锁服务
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");    //模式设置   offboard
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    offb_set_mode.request.custom_mode = "OFFBOARD";    // 模式设置
    arm_cmd.request.value = true;                           // 解锁
}

/*
********订阅轨迹话题
*/
void control::planpath_sub(void)
{
    path_sub = nh.subscribe<nav_msgs::Path>("/tra_generation", 10,trajectory_cb);     //订阅轨迹消息
} 

geometry_msgs::TwistStamped control::PID_vel(geometry_msgs::PoseStamped cuur_pose , geometry_msgs::PoseStamped targ_pose)
{
    geometry_msgs::TwistStamped _vel;
    _vel.header.frame_id = "/map";
    _vel.header.stamp = ros::Time::now();

    curr_error.x = cuur_pose.pose.position.x - targ_pose.pose.position.x;
    curr_error.y = cuur_pose.pose.position.y - targ_pose.pose.position.y;
    curr_error.z = cuur_pose.pose.position.z - targ_pose.pose.position.z;

    _vel.twist.linear.x = curr_error.x * kp;
    _vel.twist.linear.y = curr_error.y * kp;
    _vel.twist.linear.z = curr_error.z * kp;
    // std::cout << "_vel:"  << _vel <<std::endl;
    return _vel;
}

/*
*********************无人机路径跟随
*/
void control::px4_follow(void)
{
    geometry_msgs::PoseStamped pose;
    nav_msgs::Path real_path;
    pose.header.frame_id = "/map";
    pose.header.stamp = ros::Time::now();
    real_path.header.frame_id = "/map";
    real_path.header.stamp = ros::Time::now();
    ros::Rate rate(60); 
    for(int i=0; i < plan_tra.poses.size(); i++)
    {
        
        pose.pose.position.x = px4_pose.pose.position.x;
        pose.pose.position.y = px4_pose.pose.position.y;
        pose.pose.position.z = px4_pose.pose.position.z;
        // std::cout << "pose:  " << pose << std::endl;
        real_path.poses.push_back(pose);
        realpath_pub.publish(real_path);

        pose.pose.position.x = plan_tra.poses[i].pose.position.x;
        pose.pose.position.y = plan_tra.poses[i].pose.position.y;
        pose.pose.position.z = plan_tra.poses[i].pose.position.z;

        vel = PID_vel(px4_pose , plan_tra.poses[i]);
        local_vel_pub.publish(vel);
        // local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
}
 /*
 *****px4 控制
 */
void control::px4_control(void)
{
    kp = -2.5;
    px4_init();
    planpath_sub();
    while(1)
    {
        mode_set();
        px4_follow();
        ros::spinOnce();
    }
    
}

 /*
 **     main：节点初始化
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node"); 
    control control;
    
    return 0;
}
