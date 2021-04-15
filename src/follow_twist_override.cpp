//included packages
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <gazebo_msgs/ModelStates.h>
#include "../PID/cpp/PID.h"

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
double catchup_vel_offset = 5.0;
double lead_vel; //retrieved from lead car speed parameter

//max considered separation INVERTED
//double min_pid_src = 5;  //smaller than 5 and the vehicles collide
//double max_pid_src = sep_target + 100;

//PID constants
double P = 15;
double I = 0;
double D = 7;
double pid_source = 0.0;  //a1_a2_separation - sep_target;
double pid_target = 0.0;
//PID controller object

void cmdVel(double v)
{
  geometry_msgs::Twist vel;

  vel.linear.x = v;
  vel.angular.z = cmd_turn;

  pub_vel.publish(vel);
  ROS_INFO("Published Velocity: %f", vel.linear.x);
  ROS_INFO("Distance between cars: %f", a1_a2_separation);
}

void recvLeadVel(const geometry_msgs::Twist& msg){
  geometry_msgs::Twist follow_vel;
  max_vel = follow_vel.linear.x + catchup_vel_offset;
  min_vel = follow_vel.linear.x - catchup_vel_offset;
  ROS_INFO("read lead vel");
}

//PID input
double pidDoubleSource()
{
  ROS_INFO("PID double source: %f", pid_source);
  return pid_source;
}
//PID output
void pidDoubleOutput(double output)
{
  cmdVel(output);
}

unsigned long timeFunction()
{
  // Multiply by 1000 to convert seconds to milliseconds
  return (unsigned long) ros::Time::now().toSec();
}

//initializes PID components
void initPID(PIDController<double>& myDoublePIDControllerPtr){
  //myDoublePIDControllerPtr.registerTimeFunction(timeFunction);
  myDoublePIDControllerPtr.setTarget(pid_target);
  myDoublePIDControllerPtr.setOutputBounded(true);
  myDoublePIDControllerPtr.setOutputBounds(min_vel, max_vel);
  //myDoublePIDControllerPtr.setInputBounded(true);
  //myDoublePIDControllerPtr.setInputBounds(min_pid_src, max_pid_src);
  myDoublePIDControllerPtr.setEnabled(true);
}

//finds pythagorean distance between two xy points
double cartDistance(double x1, double x2, double y1, double y2)
{
  double xDiff = pow(x1-x2,2);
  double yDiff = pow(y1-y2, 2);

  return sqrt(xDiff+yDiff);
}

//Every time steering command is recieved, also transmit a velocity command
void recvThr(const geometry_msgs::Twist& msg){
  cmd_turn = msg.angular.z;
}

//Reads Gazebo topics which advertise model's positions
//http://docs.ros.org/en/jade/api/gazebo_msgs/html/msg/ModelStates.html
void recvModelStates(const gazebo_msgs::ModelStates& msg){
  int arraySize = msg.name.size();    //vehicle models are at end
  //last is lead car?
  //second last is follow car?
  double a1x = msg.pose[arraySize-2].position.x;
  double a1y = msg.pose[arraySize-2].position.y;
  double a1z = msg.pose[arraySize-2].position.z;

  double a2x = msg.pose[arraySize-1].position.x;
  double a2y = msg.pose[arraySize-1].position.y;
  double a2z = msg.pose[arraySize-1].position.z;

  a1_a2_separation = cartDistance(a1x, a2x, a1y, a2y);

  pid_source = sep_target - a1_a2_separation;

  //ROS_INFO("recvModelStates: %f", a1_a2_separation);
}

//init PID controller
PIDController<double> vel_PID_controller(P, I, D, pidDoubleSource, pidDoubleOutput);

void PIDTimerCallback(const ros::TimerEvent& event){
  vel_PID_controller.tick();
  ROS_INFO("PID ticked");
  ROS_INFO("PID Target: %f", vel_PID_controller.getTarget());
  ROS_INFO("PID error: %f", vel_PID_controller.getError());
  ROS_INFO("PID output: %f", vel_PID_controller.getOutput());
  ROS_INFO("PID feedback: %f", vel_PID_controller.getFeedback());
}

//main function
int main(int argc, char** argv){
//init section
  ros::init(argc,argv,"follow_twist_override");
  ros::NodeHandle nh;

  PIDController<double> *ip = &vel_PID_controller;
  initPID(*ip);

  nh.getParam("/a2/speed",lead_vel);

  ros::Subscriber sub_cmd_vel = nh.subscribe("/a1/cmd_vel", 1, recvThr);
  ros::Subscriber sub_lead_vel = nh.subscribe("a2/twist", 1, recvLeadVel);

  ros::Timer PID_timer = nh.createTimer(ros::Duration(0.01), PIDTimerCallback);

  pub_vel = nh.advertise<geometry_msgs::Twist>("/a1/cmd_vel", 1); //x velocity with float64
  ros::Subscriber sub_gazebo_spy = nh.subscribe("/gazebo/model_states", 1, recvModelStates);

  ros::spin();
}