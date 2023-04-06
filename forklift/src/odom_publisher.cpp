/*
 * Automatic Addison
 * Date: May 20, 2021
 * ROS Version: ROS 1 - Melodic
 * Website: https://automaticaddison.com
 * Publishes odometry information for use with robot_pose_ekf package.
 *   This odometry information is based on wheel encoder tick counts.
 * Subscribe: ROS node that subscribes to the following topics:
 *  right_ticks : Tick counts from the right motor encoder (std_msgs/Int16)
 * 
 *  left_ticks : Tick counts from the left motor encoder  (std_msgs/Int16)
 * 
 *  initial_2d : The initial position and orientation of the robot.
 *               (geometry_msgs/PoseStamped)
 *
 * Publish: This node will publish to the following topics:
 *  odom_data_euler : Position and velocity estimate. The orientation.z 
 *                    variable is an Euler angle representing the yaw angle.
 *                    (nav_msgs/Odometry)
 *  odom_data_quat : Position and velocity estimate. The orientation is 
 *                   in quaternion format.
 *                   (nav_msgs/Odometry)
 * Modified from Practical Robotics in C++ book (ISBN-10 : 9389423465)
 *   by Lloyd Brombach
 */
 
// Include various libraries
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>
 
// Create odometry data publishers
ros::Publisher odom_data_pub;
ros::Publisher odom_data_pub_quat;
nav_msgs::Odometry odomNew;
nav_msgs::Odometry odomOld;
 
// Initial pose
const double initialX = 0.0;
const double initialY = 0.0;
const double initialTheta = 0.00000000001;
const double PI = 3.141592;
 
// Robot physical constants
const double TICKS_PER_REVOLUTION = 577; // For reference purposes.
const double WHEEL_RADIUS = 0.055; // Wheel radius in meters
const double WHEEL_BASE = 0.17; // Center of left tire to center of right tire
const double TICKS_PER_METER = 1667; // Original was 2800
 
// Distance both wheels have traveled
double distanceLeft = 0;
double distanceRight = 0;
 
// Flag to see if initial pose has been received
bool initialPoseRecieved = true;
 
using namespace std;
 
// Get initial_2d message from either Rviz clicks or a manual pose publisher
void set_initial_2d(const geometry_msgs::PoseStamped &rvizClick) {
 
  odomOld.pose.pose.position.x = rvizClick.pose.position.x;
  odomOld.pose.pose.position.y = rvizClick.pose.position.y;
  odomOld.pose.pose.orientation.z = rvizClick.pose.orientation.z;
  initialPoseRecieved = true;
}
 
// Calculate the distance the left wheel has traveled since the last cycle
void Calc_Left(const std_msgs::Int16& leftCount) {
 
  static int lastCountL = 0;
  if(leftCount.data != 0 && lastCountL != 0) {
         
    int leftTicks = (leftCount.data - lastCountL);
 
    if (leftTicks > 10000) {
      leftTicks = 0 - (65535 - leftTicks);
    }
    else if (leftTicks < -10000) {
      leftTicks = 65535-leftTicks;
    }
    else{}
    distanceLeft = leftTicks/TICKS_PER_METER;
  }
  lastCountL = leftCount.data;
}
 
// Calculate the distance the right wheel has traveled since the last cycle
void Calc_Right(const std_msgs::Int16& rightCount) {
   
  static int lastCountR = 0;
  if(rightCount.data != 0 && lastCountR != 0) {
 
    int rightTicks = rightCount.data - lastCountR;
     
    if (rightTicks > 10000) {
      distanceRight = (0 - (65535 - distanceRight))/TICKS_PER_METER;
    }
    else if (rightTicks < -10000) {
      rightTicks = 65535 - rightTicks;
    }
    else{}
    distanceRight = rightTicks/TICKS_PER_METER;
  }
  lastCountR = rightCount.data;
}
 
// Publish a nav_msgs::Odometry message in quaternion format
void publish_quat() {
 
  tf2::Quaternion q;
         
  q.setRPY(0, 0, odomNew.pose.pose.orientation.z);
 
  nav_msgs::Odometry quatOdom;
  quatOdom.header.stamp = odomNew.header.stamp;
  quatOdom.header.frame_id = "odom";
  quatOdom.child_frame_id = "base_link";
  quatOdom.pose.pose.position.x = odomNew.pose.pose.position.x;
  quatOdom.pose.pose.position.y = odomNew.pose.pose.position.y;
  quatOdom.pose.pose.position.z = odomNew.pose.pose.position.z;
  quatOdom.pose.pose.orientation.x = q.x();
  quatOdom.pose.pose.orientation.y = q.y();
  quatOdom.pose.pose.orientation.z = q.z();
  quatOdom.pose.pose.orientation.w = q.w();
  quatOdom.twist.twist.linear.x = odomNew.twist.twist.linear.x;
  quatOdom.twist.twist.linear.y = odomNew.twist.twist.linear.y;
  quatOdom.twist.twist.linear.z = odomNew.twist.twist.linear.z;
  quatOdom.twist.twist.angular.x = odomNew.twist.twist.angular.x;
  quatOdom.twist.twist.angular.y = odomNew.twist.twist.angular.y;
  quatOdom.twist.twist.angular.z = odomNew.twist.twist.angular.z;
 
  for(int i = 0; i<36; i++) {
    if(i == 0 || i == 7 || i == 14) {
      quatOdom.pose.covariance[i] = .01;
     }
     else if (i == 21 || i == 28 || i== 35) {
       quatOdom.pose.covariance[i] += 0.1;
     }
     else {
       quatOdom.pose.covariance[i] = 0;
     }
  }
 
  odom_data_pub_quat.publish(quatOdom);
}
 
// Update odometry information
void update_odom() {
 
  // Calculate the average distance
  double cycleDistance = (distanceRight + distanceLeft) / 2;
   
  // Calculate the number of radians the robot has turned since the last cycle
  double cycleAngle = asin((distanceRight-distanceLeft)/WHEEL_BASE);
 
  // Average angle during the last cycle
  double avgAngle = cycleAngle/2 + odomOld.pose.pose.orientation.z;
     
  if (avgAngle > PI) {
    avgAngle -= 2*PI;
  }
  else if (avgAngle < -PI) {
    avgAngle += 2*PI;
  }
  else{}
 
  // Calculate the new pose (x, y, and theta)
  odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + cos(avgAngle)*cycleDistance;
  odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + sin(avgAngle)*cycleDistance;
  odomNew.pose.pose.orientation.z = cycleAngle + odomOld.pose.pose.orientation.z;
 
  // Prevent lockup from a single bad cycle
  if (isnan(odomNew.pose.pose.position.x) || isnan(odomNew.pose.pose.position.y)
     || isnan(odomNew.pose.pose.position.z)) {
    odomNew.pose.pose.position.x = odomOld.pose.pose.position.x;
    odomNew.pose.pose.position.y = odomOld.pose.pose.position.y;
    odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z;
  }
 
  // Make sure theta stays in the correct range
  if (odomNew.pose.pose.orientation.z > PI) {
    odomNew.pose.pose.orientation.z -= 2 * PI;
  }
  else if (odomNew.pose.pose.orientation.z < -PI) {
    odomNew.pose.pose.orientation.z += 2 * PI;
  }
  else{}
 
  // Compute the velocity
  odomNew.header.stamp = ros::Time::now();
  odomNew.twist.twist.linear.x = cycleDistance/(odomNew.header.stamp.toSec() - odomOld.header.stamp.toSec());
  odomNew.twist.twist.angular.z = cycleAngle/(odomNew.header.stamp.toSec() - odomOld.header.stamp.toSec());
 
  // Save the pose data for the next cycle
  odomOld.pose.pose.position.x = odomNew.pose.pose.position.x;
  odomOld.pose.pose.position.y = odomNew.pose.pose.position.y;
  odomOld.pose.pose.orientation.z = odomNew.pose.pose.orientation.z;
  odomOld.header.stamp = odomNew.header.stamp;
    ROS_INFO("%d \n",odomNew.pose.pose.position.x);
  // Publish the odometry message
  odom_data_pub.publish(odomNew);
}
 
int main(int argc, char **argv) {
   
  // Set the data fields of the odometry message
  odomNew.header.frame_id = "odom";
  odomNew.pose.pose.position.z = 0;
  odomNew.pose.pose.orientation.x = 0;
  odomNew.pose.pose.orientation.y = 0;
  odomNew.twist.twist.linear.x = 0;
  odomNew.twist.twist.linear.y = 0;
  odomNew.twist.twist.linear.z = 0;
  odomNew.twist.twist.angular.x = 0;
  odomNew.twist.twist.angular.y = 0;
  odomNew.twist.twist.angular.z = 0;
  odomOld.pose.pose.position.x = initialX;
  odomOld.pose.pose.position.y = initialY;
  odomOld.pose.pose.orientation.z = initialTheta;
 
  // Launch ROS and create a node
  ros::init(argc, argv, "ekf_odom_pub");
  ros::NodeHandle node;
 
  // Subscribe to ROS topics
  ros::Subscriber subForRightCounts = node.subscribe("right_tick", 100, Calc_Right, ros::TransportHints().tcpNoDelay());
  ros::Subscriber subForLeftCounts = node.subscribe("left_tick", 100, Calc_Left, ros::TransportHints().tcpNoDelay());
//   ros::Subscriber subInitialPose = node.subscribe("initial_2d", 1, set_initial_2d);
 
  // Publisher of simple odom message where orientation.z is an euler angle
  odom_data_pub = node.advertise<nav_msgs::Odometry>("odom_data_euler", 100);
 
  // Publisher of full odom message where orientation is quaternion
  odom_data_pub_quat = node.advertise<nav_msgs::Odometry>("odom", 100);
 
  ros::Rate loop_rate(30); 
     
  while(ros::ok()) {
     
    if(initialPoseRecieved) {
      update_odom();
      publish_quat();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
 
  return 0;
}
// #include <ros/ros.h>
// #include <std_msgs/Int64.h>
// #include <geometry_msgs/Quaternion.h>
// #include <tf/transform_broadcaster.h>
// #include <nav_msgs/Odometry.h>
// #include <iostream>


// #define TICK_PER_METER 4845;
// #define WHEEL_TRACK 0.5;
// long tickA;
// long tickB;

// void right_tick(const std_msgs::Int64ConstPtr &msg){
//     tickA = msg->data;
// }

// void left_tick(const std_msgs::Int64ConstPtr &msg){
//     tickB = msg->data;
// }



// int main(int argc, char* argv[])
// {

//     ros::init(argc, argv, "odom_publisher");
//     ros::NodeHandle nh;

//     ros::Subscriber subA = nh.subscribe<std_msgs::Int64>("/right_tick", 1000, right_tick);
//     ros::Subscriber subB = nh.subscribe<std_msgs::Int64>("/left_tick", 1000, left_tick);

//     ros::Publisher odomPub = nh.advertise<nav_msgs::Odometry>("odom", 1000);

//     double last_time = 0.0;
//     long prev_tickA = 0;
//     long prev_tickB = 0;
//     float th = 0.0;

//     auto now = ros::Time::now().toSec();

//     static tf::TransformBroadcaster odomBroadcaster;

//     double dx, dy, x, y;
//     while (ros::ok()){
//         auto now = ros::Time::now().toSec();
        
//         if (now - last_time > 1.0){
            
//             double dt = now - last_time;
            
//             last_time = now;

//             float dright = (tickA - prev_tickA)/1400.0;
//             float dleft = (tickB - prev_tickB)/1400.0;

            


//             prev_tickA = tickA;
//             prev_tickB = tickB;

            

//             float dxy_ave = (dright + dleft)/2.0;
//             float dth = (dright - dleft)/WHEEL_TRACK;
//             float vxy = dxy_ave / dt;
//             float vth = dth / dt;

            

//             if ( dxy_ave != 0){
//                 dx = cos(dth) * dxy_ave;
//                 dy =  -sin(dth) * dxy_ave;
//                 x += (cos(th) * dx - sin(th));
//                 y += (cos(th) * dy + sin(th));         
//             }

//             if ( dth !=0){
//                 th += dth;
//             }

//             geometry_msgs::Quaternion quaternion;
//             quaternion.x = 0.0;
//             quaternion.y = 0.0;
//             quaternion.z = sin(th / 2.0);
//             quaternion.w = cos(th / 2.0);

//             tf::Transform transform;
//             transform.setOrigin( tf::Vector3(x, y, 0.0) );
//             tf::Quaternion q;
//             q.setRPY(0, 0, th);
//             transform.setRotation(q);
//             odomBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", "odom"));

//             nav_msgs::Odometry odom;
//             odom.header.frame_id = "odom";
//             odom.child_frame_id = "base_footprint";
//             odom.header.stamp = ros::Time::now();
//             odom.pose.pose.position.x = x;
//             odom.pose.pose.position.y = y;
//             odom.pose.pose.position.z = 0;
//             odom.pose.pose.orientation = quaternion;
//             odom.twist.twist.linear.x = vxy;
//             odom.twist.twist.linear.y = 0;
//             odom.twist.twist.angular.z = vth;

//             odomPub.publish(odom);

//             ROS_INFO("x: %f ", x);
//             ROS_INFO("y: %f \n", y);
            
//         }
//         ros::spinOnce();
//     }
    
    
    
// }