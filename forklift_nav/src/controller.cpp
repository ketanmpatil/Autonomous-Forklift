#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <vector>
#include <tf/transform_datatypes.h>



class Controller{
    protected:
        ros::NodeHandle nh;
        ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 10, &Controller::odom_callback,this);
        ros::Publisher cmd = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    public:

        geometry_msgs::Twist velocity;
        nav_msgs::Odometry odom;

        float x_current, y_current;
        double theta_current;
        float x_goal = 1, y_goal = 0, theta_goal = 0;
        

        double last = ros::Time().now().toSec();
        float E_d = 0;
        float E_i = 0;

        float Kp = 1.5;
        float Ki = 0.01;
        float Kd = 0.5;

        float wl=0;
        float wr = 0;

    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom){
        double dump1 ,dump2;
        x_current = odom->pose.pose.position.x;
        y_current = odom->pose.pose.position.y;


        tf::Quaternion quat;

        geometry_msgs::Quaternion Q;
        Q.w = odom->pose.pose.orientation.w;
        Q.x = odom->pose.pose.orientation.x;
        Q.y = odom->pose.pose.orientation.y;
        Q.z = odom->pose.pose.orientation.z;

        tf::quaternionMsgToTF(Q, quat);

        tf::Matrix3x3(quat).getRPY(dump1, dump2, theta_current);

        // theta_current = odom->pose.pose.orientation.w;
    }

    void controller(){
        double now = ros::Time().now().toSec();
        float dt = now - last;

        if (dt >= 1.0){
            float error_x = x_current - x_goal;
            float error_y = y_current - y_goal;
            // float error_theta = theta_current - theta_goal;


            // auto theta_g = math::atan2(error_x, error_y);
            ROS_INFO("theta_current: %f", this->theta_current);
        
            float e_k = theta_goal - theta_current;

            e_k = atan2(sin(e_k), cos(e_k));

            ROS_INFO("Goal: %f", e_k);

            if (e_k < 0.1){
                    velocity.linear.x = 0.0;
                    velocity.angular.z = 0;

                    cmd.publish(velocity);
                    
            }

            else{

            
            
            // # Error for the proportional term
            float e_P = e_k;
            
            // # Error for the integral term.
            float e_I = E_i + e_k * dt;
                    
            // # Error for the derivative term.
            float e_D = (e_k - E_d)/dt;

            float w = Kp * e_P + Ki * e_I + Kd * e_D; 

            w = w > 0.6 ? 0.6 : w;
            w = w < -0.6 ? -0.6 : w;

            // getIndividualVelocities(w);
            velocity.linear.x = 0.2;
            velocity.angular.z = w;

            cmd.publish(velocity);

            E_i = e_I;
            E_d = e_D;
            
            }

  
            last = now;
        }


    }
    void getIndividualVelocities(float &w){
        wl = (0.2 - w*(0.3)/2)/0.055;
        wr = (0.2 + w*(0.3)/2)/0.055;
    }
        
        

};

int main(int argc, char** argv){
    ros::init(argc, argv, "controller");
    // ros::NodeHandle nh;

    Controller controller;

    while (ros::ok()){
        controller.controller();
    }
}