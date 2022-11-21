#include "control.h"
//建立一个订阅消息体类型的变量，用于存储订阅的信息
mavros_msgs::State current_state;
nav_msgs::Path plan_tra;


//订阅时的回调函数，接受到该消息体的内容时执行里面的内容，这里面的内容就是赋值
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
    std::cout << "current_state:  " << current_state << std::endl;
}
 
 void trajectory_cb(const nav_msgs::Path::ConstPtr &wp)
{
    plan_tra = *wp;
}

/*
**       初始化px4，通信连接、解锁、模式设置
*/
void control::px4_init(void)
{
    ros::Rate rate(20.0);  //设置发布速率
    state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);     //订阅飞机状态
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
**订阅轨迹话题
*/
void control::planpath_sub(void)
{
    path_sub = nh.subscribe<nav_msgs::Path>("/tra_generation", 10,trajectory_cb);     //订阅轨迹消息
    // ros::spin();
} 

void control::px4_follow(void)
{
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "/map";
    pose.header.stamp = ros::Time::now();
    ros::Rate rate(99); 
    for(int i=0; i < plan_tra.poses.size(); i++)
    {
        pose.pose.position.x = plan_tra.poses[i].pose.position.x;
        pose.pose.position.y = plan_tra.poses[i].pose.position.y;
        pose.pose.position.z = plan_tra.poses[i].pose.position.z;
        local_pos_pub.publish(pose);
        rate.sleep();
    }
    while(1)
    {
        local_pos_pub.publish(pose);
    }
}
 /*
 *****px4 控制
 */
void control::px4_control(void)
{
    px4_init();
    planpath_sub();
    while(1)
    {
        if( current_state.mode != "OFFBOARD")
                if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                    // ROS_INFO("Offboard enabled");//打开模式后打印信息
                }
        else //else指已经为offboard模式，然后进去判断是否解锁，如果没有解锁，则客户端arming_client向服务端arm_cmd发起请求call
                //然后服务端回应response成功解锁，这就解锁了
        {
            if( !current_state.armed)
                if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");//解锁后打印信息
                }
        }
        // if(current_state.mode == "OFFBOARD" && current_state.armed) break;
        if( current_state.armed) 
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
while (1)
{
    // std::cout <<"plan_tra:" << control::plan_tra << std::endl;
    
}

 
    return 0;
}
