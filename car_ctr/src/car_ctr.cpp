#include "car_ctr.hpp"

/** 
* @brief   限幅函数 
* @remarks min<data<max
*/
float Limit(float data,float max,float min){
	float Temp = data;
	if(data >= max) Temp = max;
	if(data <= min) Temp = min;
	return Temp;
}

float PID_t::Run(float err){
	CurrentError = err;
	Pout = CurrentError * P;
	
	if(abs(CurrentError) < I_Threshold)  Iout += I * CurrentError;
	else  Iout=0;
	Iout= Limit(Iout,IMax,-IMax);  
	if(abs(CurrentError) <= I_DeadArea)  Iout = 0;  //清除积分残留值，防止在无误差下依然输出
	
	// Dout = (1 - Beta) * D * (CurrentError - LastError) + Beta * LastDout; 
	// Dout= Limit(Dout, IMax, -IMax);  

	PIDout = Limit(Pout + Iout, PIDMax, -PIDMax);  //输出限幅
	
	LastError = CurrentError;
	return PIDout;
}

Car_Ctr::Car_Ctr(ros::NodeHandle& nh){

    nh.param<int>("car_speed", _car_speed_, 0);
    nh.param<float>("angular_p", _angular_p_, 10.0);
    nh.param<float>("angular_i", _angular_i_, 0.12);
    nh.param<float>("angular_d", _angular_d_, 0);
    nh.param<float>("angular_imax", _angular_imax_, 100);
    nh.param<float>("angular_pidmax", _angular_pidmax_, 250);
    nh.param<float>("angular_ithreshold", _angular_ithreshold_, 15);
    nh.param<float>("angular_ideadarea", _angular_ideadarea_, 0.5);
    nh.param<string>("follow_path", _follow_path_, "/move_base/NavfnROS/plan");
    // std::cout << _follow_path_ << std::endl;
  
    car_odom_sub = nh.subscribe("/vins_estimator/odometry", 10, &Car_Ctr::car_odom_Callback, this);
    _legoloam_sub_ = nh.subscribe("/integrated_to_init", 10, &Car_Ctr::_legoloamCallback, this);

    _goal_sub_ = nh.subscribe("/move_base_simple/goal", 1, &Car_Ctr::_goalCallback, this);
    
    _path_sub_ = nh.subscribe(_follow_path_, 1, &Car_Ctr::_pathCallback, this);
    // _path_sub_ = nh.subscribe("/global_path_rough", 10, &Car_Ctr::_pathCallback, this);
    // _path_sub_ = nh.subscribe("/stand_local_path", 10, &Car_Ctr::_pathCallback, this);
    
    // d435i相机坐标系到无人机机体坐标系
    T_cb.setOrigin(tf::Vector3(0,0,0));
    T_cb.setRotation(tf::createQuaternionFromRPY(0, 0, 0));
    // d435i相机可能装的很歪，或者是不是超前，则可以在这里增加转换
    T_wo.setOrigin(tf::Vector3(0,0,0));
    T_wo.setRotation(tf::createQuaternionFromRPY(0, 0.0/180.0*M_PI, 0));

    // pid_yaw = new PID_t(10.0, 0.12, 0, 100, 250, 15, 0.5);  // 小于15度开始积分，小于0.5度和大于15度不积分
    pid_yaw = new PID_t(_angular_p_, _angular_i_, _angular_d_, _angular_imax_, _angular_pidmax_, _angular_ithreshold_, _angular_ideadarea_);
    // std::cout << _angular_p_ << " " << _angular_i_ << " " << _angular_d_ << " " << _angular_imax_ << " " 
    //     << _angular_pidmax_ << " " << _angular_ithreshold_ << " " << _angular_ideadarea_ << std::endl;
    
    try{  // 打开串口
        ser.setPort("/dev/carcontrol_uart");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
        ROS_INFO("serial opened");
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return;
    }
    car_ctr_timer = nh.createTimer(ros::Duration(1.0/50.0), &Car_Ctr::car_ctr_timer_cb, this);
    upper_ctr_timer = nh.createTimer(ros::Duration(1.0/10.0), &Car_Ctr::upper_ctr_timer2_cb, this);
}

void Car_Ctr::speed_set(int stright_vel, int omega){
    state.target_vel = stright_vel;
    state.target_yaw = omega*M_PI/180.0;
}

//底层控制，将控制指令通过串口发送给底盘
void Car_Ctr::car_ctr_timer_cb(const ros::TimerEvent& event){
    //tempErr0、tempErr1互为正负、和为2Pi
    float tempErr0 = state.target_yaw - state.real_yaw;
    float tempErr1 = tempErr0 - 2 * M_PI * SIGN(tempErr0);
    // ROS_INFO("tempErr0: %.2f, tempErr1: %.2f", tempErr0, tempErr1);
    float target_yaw_rate = pid_yaw->Run(- 180.0 / M_PI * (abs(tempErr0)>abs(tempErr1)?tempErr1:tempErr0));
    // ROS_INFO("target_yaw: %.2f, real_yaw: %.2f, yaw_rate: %.2f", state.target_yaw * 180.0 / M_PI, state.real_yaw * 180.0 / M_PI, target_yaw_rate);
    std::string driver_data = "!M " + std::to_string(-state.target_vel) + ' ' + std::to_string(target_yaw_rate)+ '\r';
    if(state.if_odom_ready && !reach_flag) ser.write(driver_data);
}

//上层控制 
void Car_Ctr::upper_ctr_timer_cb(const ros::TimerEvent& event){
    //给定目标点集
    // static std::vector<float> tgt_path_x = {0.0, 0.0, 27.0,27.0};
    // static std::vector<float> tgt_path_y = {0.0,-22.0,-22.0,0.0 };
    // //计算目标点与车位置在全局坐标系（x朝前右手坐标系）中与x轴正方向的角度
    // float target_yaw = atan2(tgt_path_y.back() - car_y, tgt_path_x.back() - car_x);
    // //计算目标点与车的距离 
    // float dist = sqrt(pow(tgt_path_y.back() - car_y,2)+pow(tgt_path_x.back() - car_x,2));
    // //如果距离小于0.5，可认为已经到该点，跟踪下一个点
    // if(dist < 0.5){
    //     tgt_path_x.pop_back();
    //     tgt_path_y.pop_back();
    // }
    //当接收到路径点集后，开始控制轨迹跟踪
    if(receive_path_flag){
        float target_yaw = atan2(path_y.back() - car_y, path_x.back() - car_x);

        float dist = sqrt(pow(path_y.back() - car_y,2)+pow(path_x.back() - car_x,2));

        if(dist < 0.5){
            path_x.pop_back();
            path_y.pop_back();
        }
        //线速度始终300，将目标角度发给控制接口
        speed_set(300, target_yaw*180.0/M_PI);

        //当点集为空，到达目标点,清空路径点集
        if(path_x.empty()){
            reach_flag = true;
            receive_path_flag = false;
        } 
    }
    
}

void Car_Ctr::upper_ctr_timer2_cb(const ros::TimerEvent& event){
    // std::cout << goal_x << " " << goal_y << " " << if_have_goal << std::endl;
    float target_yaw;
    float dist;
    //std::cout << reach_flag << std::endl;
    if(if_have_goal){
        std::cout << "next_x: " << planning_path.poses[20].pose.position.x << " next_y: " << 
            planning_path.poses[20].pose.position.y << " car_x: " << car_x << " car_y: " << 
            car_y << " goal_x: " << goal_x << " goal_y: " << goal_y <<  
            " if_have_goal: " << if_have_goal << " reach_flag: " << reach_flag <<std::endl;
        target_yaw= atan2(planning_path.poses[20].pose.position.y - car_y, planning_path.poses[20].pose.position.x - car_x);

        //线速度始终300，将目标角度发给控制接口
        speed_set(_car_speed_, target_yaw*180.0/M_PI);
    }

    dist = sqrt(pow(goal_y - car_y,2)+pow(goal_x - car_x,2));

    if(dist < 1.0){
        reach_flag = true;
        if_have_goal = false;
    } 
}   

//视觉惯性里程计 反馈车位姿：car_x car_y state.real_yaw x朝前右手坐标系
void Car_Ctr::car_odom_Callback(const nav_msgs::Odometry::ConstPtr& msg){
    tf::Matrix3x3 mat(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
    geometry_msgs::Vector3 vins_euler;
    mat.getRPY(vins_euler.x, vins_euler.y, vins_euler.z);

    /* 无人机位置转换 */
    car_odom.header.frame_id = "odom";      // 坐标tf必须这样写
    car_odom.child_frame_id = "base_link";
    car_odom.header.stamp = msg->header.stamp;

    tf::Transform T_oc, T_wb;  // T_oc为里程计坐标系到相机坐标系的变换矩阵
    T_oc.setOrigin(tf::Vector3(msg->pose.pose.position.y, -msg->pose.pose.position.x, msg->pose.pose.position.z));
    T_oc.setRotation(tf::createQuaternionFromRPY(vins_euler.y, -(vins_euler.x + M_PI/2), vins_euler.z));
    T_wb = T_wo * T_oc * T_cb;

    car_odom.pose.pose.position.x = T_wb.getOrigin().x();
    car_odom.pose.pose.position.y = T_wb.getOrigin().y();
    car_odom.pose.pose.position.z = T_wb.getOrigin().z();
    car_odom.pose.pose.orientation.x = T_wb.getRotation().x();
    car_odom.pose.pose.orientation.y = T_wb.getRotation().y();
    car_odom.pose.pose.orientation.z = T_wb.getRotation().z();
    car_odom.pose.pose.orientation.w = T_wb.getRotation().w();

    //车位姿
    state.real_yaw = (float)tf::getYaw(car_odom.pose.pose.orientation);
    car_x = car_odom.pose.pose.position.x;
    car_y = car_odom.pose.pose.position.y;
    // std::cout << car_x << " " << car_y << " " << state.real_yaw*180.0/M_PI << std::endl;
    state.if_odom_ready = true; 
}

//激光里程计 反馈车位姿：car_x car_y state.real_yaw x朝前右手坐标系
void Car_Ctr::_legoloamCallback(const nav_msgs::Odometry::ConstPtr& msg){
    tf::Matrix3x3 mat(tf::Quaternion(msg->pose.pose.orientation.z, -msg->pose.pose.orientation.x, 
        -msg->pose.pose.orientation.y, msg->pose.pose.orientation.w));
    geometry_msgs::Vector3 vins_euler;
    mat.getRPY(vins_euler.x, vins_euler.y, vins_euler.z);

    car_x = msg->pose.pose.position.z;
    car_y = msg->pose.pose.position.x;
    state.real_yaw = -vins_euler.z;

    // std::cout << car_x << " " << car_y << " " << state.real_yaw*180.0/M_PI << std::endl;
    state.if_odom_ready = true;
}

void Car_Ctr::_pathCallback(const nav_msgs::Path::ConstPtr& msg){
    planning_path = *msg;
    // path_x.clear();
    // path_y.clear();
    //receive_path_flag = true;
    // for(int i=0;i<msg->poses.size();i++){
    //     path_x.push_back(msg->poses[i].pose.position.x);
    //     path_y.push_back(msg->poses[i].pose.position.y);
    // }
    // for(int i=0;i<path_x.size();i++){
    //     std::cout << path_x[i] << " " << path_y[i] << std::endl;
    // }
    // std::cout << "------------------" << std::endl;

    // path_x.clear();
    // path_y.clear();
    // // static bool state = true;
    // // state = ~state;
    // if(!msg->poses.empty()){
    //     for(int i=0;i<msg->poses.size();i++){
    //         path_x.push_back(msg->poses[i].pose.position.x);
    //         path_y.push_back(msg->poses[i].pose.position.y);
    //         // i += 20;
    //     }
    //     reverse(path_x.begin(),path_x.end());
    //     reverse(path_y.begin(),path_y.end());
    //     std::cout << path_x.size() << std::endl;
    //     for(int i=0;i<path_x.size();i++){
    //         std::cout << path_x[i] << " " << path_y[i] << std::endl;
    //     }
    //     std::cout << "------------------" << std::endl;
    // }  
}

void Car_Ctr::_goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    // if(goal_x != msg->pose.position.x && goal_y != msg->pose.position.y){
    //     reach_flag = false;
    // }
    goal_x = msg->pose.position.x;
    goal_y = msg->pose.position.y; 
    if_have_goal = true;
    reach_flag = false;
    // std::cout << "goal_x: " << goal_x << " goal_y: " << goal_y << std::endl;
}