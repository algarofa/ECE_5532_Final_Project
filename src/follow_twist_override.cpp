//included packages
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/LinkState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>


//GLOBAL VARIABLES
ros::Publisher pub_vel;
double a1_a2_separation;

//Every time steering command is recieved, also transmit a velocity command
void recvThr(const geometry_msgs::Twist& msg){
  //ROS_INFO("rcvThr:     Angular R: %f   P: %f   Y: %f   ", msg.angular.x, msg.angular.y, msg.angular.z);
  //ROS_INFO("rcvThr:     Linear  X: %f   Y: %f   Z: %f   ", msg.linear.x, msg.linear.y, msg.linear.z);

  geometry_msgs::Twist vel;

  vel.linear.x = 10.0;
  vel.angular.z = msg.angular.z;

  pub_vel.publish(vel);
  //ROS_INFO("Published Velocity: %f", vel.linear.x);
}

//http://docs.ros.org/en/jade/api/gazebo_msgs/html/msg/ModelStates.html
void recvModelStates(const gazebo_msgs::ModelStates& msg){

  ROS_INFO("number of names: %d", msg.name.size());

  ROS_INFO("Object %d X: %f", msg.pose[msg.name.size()-1].position.x);
  ROS_INFO("Object %d Y: %f", msg.pose[msg.name.size()-1].position.y);
  ROS_INFO("Object %d Z: %f", msg.pose[msg.name.size()-1].position.z);

  ROS_INFO("Object %d X: %f", msg.pose[msg.name.size()-2].position.x);
  ROS_INFO("Object %d Y: %f", msg.pose[msg.name.size()-2].position.y);
  ROS_INFO("Object %d Z: %f", msg.pose[msg.name.size()-2].position.z);

}


//main function
int main(int argc, char** argv){
//init section
  ros::init(argc,argv,"follow_twist_override");
  ros::NodeHandle nh;

  ROS_INFO("INTO MAIN");

  //ros::Timer timer = nh.createTimer(ros::Duration(1.0), speedTimerCallback);

  pub_vel = nh.advertise<geometry_msgs::Twist>("/a1/cmd_vel", 1); //x velocity with float64
  //ros::Subscriber sub_twist = nh.subscribe("/a1/twist", 1, recvTwist);
  ros::Subscriber sub_cmd_vel = nh.subscribe("/a1/cmd_vel", 1, recvThr);

  //ros::Subscriber sub_a1_joint_states = nh.subscribe("/a1/joint_states", 1, recvA1LinkStates);
  ros::Subscriber sub_a2_joint_states = nh.subscribe("/a1/joint_states", 1, recvA1LinkStates);

  ros::Subscriber sub_gazebo_spy = nh.subscribe("/gazebo/model_states", 1, recvModelStates);
  //ros::Subscriber sub_gazebo_spy2 = nh.subscribe("/gazebo/model_state", 1, recvModelState);


  ros::spin();
}