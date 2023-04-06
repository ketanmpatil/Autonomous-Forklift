#include <forklift/robot_hardware_interface.h>
#include <ros/ros.h>
#include <forklift/motor_velocity.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

float left_velocity, right_velocity;
ros::Publisher jointBpub;
ros::Publisher jointApub;


MyRobot::MyRobot(ros::NodeHandle& nh) : nh_(nh) {


// Declare all JointHandles, JointInterfaces and JointLimitInterfaces of the robot.
    init();
    
    jointApub = nh.advertise<std_msgs::Float64>("motorAvelocity", 10);
    jointBpub = nh.advertise<std_msgs::Float64>("motorBvelocity", 10);
    
    
    
// Create the controller manager
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    
//Set the frequency of the control loop.
    loop_hz_=10;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
    
//Run the control loop
    my_control_loop_ = nh_.createTimer(update_freq, &MyRobot::update, this);
}
MyRobot::~MyRobot() {
}
void MyRobot::init() {
        
// Create joint_state_interface for JointA
    hardware_interface::JointStateHandle jointStateHandleA("JointA", &joint_position[0], &joint_velocity[0], &joint_effort[0]);
    joint_state_interface_.registerHandle(jointStateHandleA);
// Create effort joint interface as JointA accepts effort command.
    hardware_interface::JointHandle jointEffortHandleA(jointStateHandleA, &joint_effort_command[0]);
    effort_joint_interface_.registerHandle(jointEffortHandleA); 
// Create Joint Limit interface for JointA
    joint_limits_interface::getJointLimits("JointA", nh_, limits);
    joint_limits_interface::EffortJointSaturationHandle jointLimitsHandleA(jointEffortHandleA, limits);
    effortJointSaturationInterface.registerHandle(jointLimitsHandleA);    
    
// Create joint_state_interface for JointB
    hardware_interface::JointStateHandle jointStateHandleB("JointB", &joint_position[1], &joint_velocity[1], &joint_effort[1]);
    joint_state_interface_.registerHandle(jointStateHandleB);
// Create effort joint interface as JointB accepts effort command..
    hardware_interface::JointHandle jointEffortHandleB(jointStateHandleB, &joint_effort_command[1]);
    effort_joint_interface_.registerHandle(jointEffortHandleB);
// Create Joint Limit interface for JointB
    joint_limits_interface::getJointLimits("JointB", nh_, limits);
    joint_limits_interface::EffortJointSaturationHandle jointLimitsHandleB(jointEffortHandleB, limits);
    effortJointSaturationInterface.registerHandle(jointLimitsHandleB);     


// Register all joints interfaces    
    registerInterface(&joint_state_interface_);
    registerInterface(&effort_joint_interface_);
    registerInterface(&effortJointSaturationInterface);
  
}

//This is the control loop
void MyRobot::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void MyRobot::read(){

  // Write the protocol (I2C/CAN/ros_serial/ros_industrial)used to get the current joint position and/or velocity and/or effort       

  //from robot.
  // and fill JointStateHandle variables joint_position_[i], joint_velocity_[i] and joint_effort_[i]
    joint_velocity[0] = right_velocity;
    joint_velocity[1] = left_velocity;


    ROS_INFO("Feed B: %f", left_velocity);
    ROS_INFO(" Feed A: %f \n", right_velocity);

}

void MyRobot::write(ros::Duration elapsed_time) {
  // Safety
  effortJointSaturationInterface.enforceLimits(elapsed_time);   // enforce limits for JointA and JointB


  // Write the protocol (I2C/CAN/ros_serial/ros_industrial)used to send the commands to the robot's actuators.
  // the output commands need to send are joint_effort_command_[0] for JointA, joint_effort_command_[1] for JointB and 

  //joint_position_command_ for JointC.
  velocityA.data = joint_effort_command[1];
  velocityB.data = joint_effort_command[0];
  jointApub.publish(velocityA);
  jointBpub.publish(velocityB);

  ROS_INFO("Vel A: %f", velocityA.data);
  ROS_INFO(" Vel B: %f \n", velocityB.data);

}

void velocityAcallback(const std_msgs::Float64ConstPtr &msg){
    left_velocity = msg->data;    
}


void velocityBcallback(const std_msgs::Float64ConstPtr &msg){
    right_velocity = msg->data;    
}

void cmd_callback(const geometry_msgs::TwistConstPtr &msg){
        float linear = msg->linear.x;
        float angular = msg->angular.z;

        std_msgs::Float64 left_speed, right_speed;

        left_speed.data = linear - angular*0.3/2;
        right_speed.data = linear + angular*0.3/2;

        if (left_speed.data > 0.33) {left_speed.data = 0.33;}
        if (left_speed.data < -0.33) {left_speed.data = -0.33;}


        if (right_speed.data > 0.33) {right_speed.data = 0.33;}
        if (right_speed.data < -0.33) {right_speed.data = -0.33;}


        jointApub.publish(left_speed);
        jointBpub.publish(right_speed);

}

int main(int argc, char** argv)
{

    //Initialze the ROS node.
    ros::init(argc, argv, "MyRobot_hardware_inerface_node");
    ros::NodeHandle nh;

    MyRobot ROBOT(nh);

    
    
    //Separate Sinner thread for the Non-Real time callbacks such as service callbacks to load controllers
    ros::MultiThreadedSpinner spinner(2) ; 

    ros::Subscriber subA = nh.subscribe("jointAvelocity" ,1000, velocityAcallback);
    ros::Subscriber subB = nh.subscribe("jointBvelocity" ,1000, velocityBcallback);
    ros::Subscriber sub2 = nh.subscribe("cmd_vel" ,1000, cmd_callback);
    jointApub = nh.advertise<std_msgs::Float64>("MyRobot/JointA_EffortController/command", 10);
    jointBpub = nh.advertise<std_msgs::Float64>("MyRobot/JointB_EffortController/command", 10);
    

    spinner.spin();
    
    return 0;
}