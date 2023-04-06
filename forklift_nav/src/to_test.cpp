#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <iostream>

class Controller{
    public: 
        double last = ros::Time().now().toSec();

    void controller(){
        double now = ros::Time().now().toSec();
        float dt = now - last;

        if (dt >= 0.1){
            std::cout << "Time: " << dt << std::endl;
            last = now;
        }
        

    }
        
        

};


int main(int argc, char** argv){
    ros::init(argc, argv, "to_test");
    ros::NodeHandle nh;
    Controller controller;
    while (ros::ok()){
        controller.controller();
    }
}