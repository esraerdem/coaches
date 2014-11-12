#include <ros/ros.h>

int main(int argc, char** argv)  {
    ros::init(argc, argv, "hello_coaches");   
    ROS_INFO("Hello COACHES developers!!!");
    ros::spinOnce();
}
