//included packages
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelStates.h>
#include "../PID/cpp/PID.h"
#include <sensor_msgs/Image.h>

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

ros::Publisher pub_thresh;

//publishes velocity and steering command messages
void cmdVel(double v)
{
  geometry_msgs::Twist vel;

  vel.linear.x = v;
  vel.angular.z = cmd_turn;

  pub_vel.publish(vel);
  //ROS_INFO("Published Velocity: %f", vel.linear.x);
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
  //last in array is lead car?
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

//refreshes PID
void PIDTimerCallback(const ros::TimerEvent& event){
  vel_PID_controller.tick();
  /*ROS_INFO("PID ticked");
  ROS_INFO("PID Target: %f", vel_PID_controller.getTarget());
  ROS_INFO("PID error: %f", vel_PID_controller.getError());
  ROS_INFO("PID output: %f", vel_PID_controller.getOutput());
  ROS_INFO("PID feedback: %f", vel_PID_controller.getFeedback()); */
}

//main function
int main(int argc, char** argv){
//init section
  ros::init(argc,argv,"follow_twist_override");
  ros::NodeHandle nh;

  //initializing PID object, passes pointer
  PIDController<double> *ip = &vel_PID_controller;
  initPID(*ip);

  //subscriber for steering messages
  ros::Subscriber sub_cmd_vel = nh.subscribe("/a1/cmd_vel", 1, recvThr);

  //timer to refresh PID
  ros::Timer PID_timer = nh.createTimer(ros::Duration(0.01), PIDTimerCallback);

  //for publishing steering and throttle messages
  pub_vel = nh.advertise<geometry_msgs::Twist>("/a1/cmd_vel", 1); //x velocity with float64

  //getting vehicle positions
  ros::Subscriber sub_gazebo_spy = nh.subscribe("/gazebo/model_states", 1, recvModelStates);

  //ros::Subscriber pic_size = nh.subscribe("/a1/front_camera/image_raw", 1, recImage);

  ros::spin();
}