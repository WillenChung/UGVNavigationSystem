#include "car_ctr.hpp"

int main(int argc, char **argv){
    ros::init(argc, argv, "car_ctr_node");
    ros::NodeHandle nh("~");
    Car_Ctr car_ctr(nh);
    ROS_INFO("car_ctr_node launched");
    ros::spin();
    return 0;
}

