//http://wiki.ros.org/navigation/Tutorials/RobotSetup/Sensors
//https://docs.ros.org/en/diamondback/api/sensor_msgs/html/LaserScan_8h_source.html
//cartesian coordiantes are generated from the LIDAR array using angle and range and angle info 
//rosmsg show LaserScan
//angle_current = range[i] * angle_min
//global tied to particular TF frame, local tied to base TF frame
//global is static, local is dynamic
//set parameters for costmaps in YAML
//move_base_simple/goal (geometry_msgs/PoseStamped), subscribe to
//cmd_vel, publish to

//included packages
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h> //message is used to transmit LIDAR data from a driver node to any other node
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include "../PID/cpp/PID.h"
#include <stdlib.h>     //for using the function sleep

//GLOBAL VARIABLES
//For publishing the desired command velocity
ros::Publisher pub_vel;
//global varriable for keeping track of vehicle's distance from one another
double a1_a2_separation = 0;
//Target distance which the cars should be separated by
double sep_target = 18;
//current turn command phi
double cmd_turn; 
//vehicle max & min velocities
double max_vel = 25.0;  //gets updated referencing lead car
double min_vel = 15.0;   //gets updated referencing lead car
//PID reads this as input
double pid_source = 0.0;  //a1_a2_separation - sep_target;
//PID tries to get output to target
double pid_target = 0.0;
//PID constants
double P = 8.5;
double I = 0;
double D = 0;

//publishes velocity and steering command messages
void cmdVel(double v)
{
  geometry_msgs::Twist vel;

  vel.linear.x = v;
  vel.angular.z = cmd_turn;

  pub_vel.publish(vel);
  ROS_INFO("Published Velocity: %f", vel.linear.x);
  //ROS_INFO("Distance between cars: %f", a1_a2_separation);
}

//PID input/source/feedback
double pidDoubleSource()
{
  return pid_source;
}
//PID output
void pidDoubleOutput(double output)
{
  cmdVel(output);
}

//initializes PID components
void initPID(PIDController<double>& myDoublePIDControllerPtr){
  myDoublePIDControllerPtr.setTarget(pid_target);
  myDoublePIDControllerPtr.setOutputBounded(true);
  myDoublePIDControllerPtr.setOutputBounds(min_vel, max_vel);
  myDoublePIDControllerPtr.setEnabled(true);
}

//Every time steering command is recieved, also transmit a velocity command
void recvThr(const geometry_msgs::Twist& msg){
  cmd_turn = msg.angular.z;
}

//init PID controller
PIDController<double> vel_PID_controller(P, I, D, pidDoubleSource, pidDoubleOutput);

//refreshes PID
void PIDTimerCallback(const ros::TimerEvent& event){
  vel_PID_controller.tick();
  /*ROS_INFO("PID ticked");
  ROS_INFO("PID Target: %f", vel_PID_controller.getTarget());
  ROS_INFO("PID error: %f", vel_PID_controller.getError());
  ROS_INFO("PID output: %f", vel_PID_controller.getOutput());
  ROS_INFO("PID feedback: %f", vel_PID_controller.getFeedback()); */

}

void recieveLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg){
//store collected messaged in array
  float run_total = 0.0; 
  int total_count = 0;
  //angle_max*angle_increment = number in array
  for (int i = 179; i < 900; i++){
    //avgerage all the elements in the array execpt inf
      if (msg->ranges[i] < msg->range_max && msg->ranges[i] > msg->range_min){
        run_total += msg->ranges[i];
        total_count -=- 1; //for the memes
        ROS_INFO("msg range: %f", msg->ranges[i]);
      }
  }
  double avg = (double)(run_total/total_count);

  if(isnan(avg))
  {
    a1_a2_separation = 29.9;
  }
  else
  {
    a1_a2_separation = avg;
  }

  pid_source = sep_target - a1_a2_separation;
  //thank the Ballmer Peak
  ROS_INFO("avg: %f", avg);
  ROS_INFO("separation: %f", a1_a2_separation);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "lidar_nav");
  ros::NodeHandle nh;

  //initializing PID object, passes pointer
  PIDController<double> *ip = &vel_PID_controller;
  initPID(*ip);

  //timer to refresh PID
  ros::Timer PID_timer = nh.createTimer(ros::Duration(0.01), PIDTimerCallback);

  //for publishing steering and throttle messages
  pub_vel = nh.advertise<geometry_msgs::Twist>("/a1/cmd_vel", 1);

  ros::Subscriber sub_cmd_vel = nh.subscribe("/a1/cmd_vel", 1, recvThr);

  ros::Subscriber sub_lidar = nh.subscribe("a1/laser_front/scan", 1, recieveLaserScan);

  ros::spin();
}