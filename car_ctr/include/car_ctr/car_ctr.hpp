#ifndef __CAR_CTR_HPP
#define __CAR_CTR_HPP
#include <ros/ros.h>
#include <iostream>
#include <serial/serial.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
using namespace std;

#define SIGN(x)  ((x)>0?1:((x)<0?-1:0))


class PID_t
{
public:
	PID_t(float _P, float _I, float _D, float _IMax, float _PIDMax, float _I_Threshold=9999, float _I_DeadArea = 0) 
		:P(_P), I(_I), D(_D), IMax(_IMax), PIDMax(_PIDMax), I_Threshold(_I_Threshold), I_DeadArea(_I_DeadArea) {}
	float Run(float err);
	float PIDMax;      //< 最大输出值
private:
	float P;
	float I;
	float D;
	float IMax;
	float I_Threshold; //< 当误差小于I_Threshold时才进行I输出
    float I_DeadArea;  //< 当误差小于I_DeadArea时不输出I
    float Beta;        //< Beta越大，受上一次的D输出影响越大，且必须0到1之间的数字
	//运算储存区
	float CurrentError;
	float LastError;
	float Pout;
	float Iout;
	float Dout;
	float LastDout;
	float PIDout;
};

class Car_Ctr{
public:
    Car_Ctr(ros::NodeHandle& nh);
    ~Car_Ctr(){
        ser.close();
        ROS_INFO("Serial Port closed");
        ROS_INFO("car_ctr_node closed");
    }
    std::vector<float> path_x;
    std::vector<float> path_y;
private:
    void speed_set(int stright_vel, int omega);
    void car_odom_Callback(const nav_msgs::Odometry::ConstPtr& msg);
    void _legoloamCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void _pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void _goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    
    void car_ctr_timer_cb(const ros::TimerEvent& event);
    void upper_ctr_timer_cb(const ros::TimerEvent& event);
    void upper_ctr_timer2_cb(const ros::TimerEvent& event);
    int culNearestIndex(float car_x, float car_y, nav_msgs::Path path);
private:
    PID_t* pid_yaw;
    ros::Subscriber car_odom_sub;     //[订阅者] 视觉惯性里程计
    ros::Subscriber _legoloam_sub_;   //[订阅者] 激光里程计
    ros::Subscriber _goal_sub_;   //[订阅者] 目标点位置
    ros::Subscriber _path_sub_;   //[订阅者] 路径点集
    ros::Timer car_ctr_timer;       //串口接受数据定时器
    ros::Timer upper_ctr_timer;
    serial::Serial ser;
    bool reach_flag = false;
    bool receive_path_flag = false;
    bool if_have_goal = false;
    struct{
        bool  if_odom_ready = false;
        float target_vel = 0;
        float target_yaw = 0;
        float real_vel = 0;
        float real_yaw = 0;   // 弧度
    }state;
    float car_x;
    float car_y;
    float goal_x = 1000;
    float goal_y = 1000;
    tf::Transform T_wo,T_cb;                   // T_cb相机坐标系到body系的变换矩阵 这里的c是realsens T_wo为世界坐标系到里程计坐标系的转换（因为里程计坐标系不一定水平）       
    nav_msgs::Odometry car_odom;
    nav_msgs::Path planning_path;
    int cout_i = 0;
    //rosparam参数
    int _car_speed_;
    float _angular_p_;
    float _angular_i_;
    float _angular_d_;
    float _angular_imax_;
    float _angular_pidmax_;
    float _angular_ithreshold_;
    float _angular_ideadarea_;
    std::string _follow_path_;  
};



#endif
