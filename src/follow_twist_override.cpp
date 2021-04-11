//included packages
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

//global variables
ros::Publisher pub_vel;

//periodically sends a speed command
/*
void speedTimerCallback(const ros::TimerEvent& event)
{
  geometry_msgs::Twist vel;

  vel.linear.x = 5.0;

  pub_vel.publish(vel);

  ROS_INFO("Published velocity");
}
*/

/*
void recvTwist(const geometry_msgs::Twist& msg){
  ROS_INFO("rcvTwist:   Angular R: %f   P: %f   Y: %f   ", msg.angular.x, msg.angular.y, msg.angular.z);
  ROS_INFO("rcvTwist:   Linear  X: %f   Y: %f   Z: %f   ", msg.linear.x, msg.linear.y, msg.linear.z);
}
*/

//Every time steering command is recieved, also transmit a velocity command
void recvThr(const geometry_msgs::Twist& msg){
  //ROS_INFO("rcvThr:     Angular R: %f   P: %f   Y: %f   ", msg.angular.x, msg.angular.y, msg.angular.z);
  //ROS_INFO("rcvThr:     Linear  X: %f   Y: %f   Z: %f   ", msg.linear.x, msg.linear.y, msg.linear.z);

  geometry_msgs::Twist vel;

  vel.linear.x = 25.0;
  vel.angular.z = msg.angular.z;

  pub_vel.publish(vel);
  //ROS_INFO("Published Velocity: %f", vel.linear.x);
}

void recvA1Joint(const sensor_msgs::JointState& msg){
  //ROS_INFO("rcvThr:     Angular R: %f   P: %f   Y: %f   ", msg.angular.x, msg.angular.y, msg.angular.z);
  //ROS_INFO("rcvThr:     Linear  X: %f   Y: %f   Z: %f   ", msg.linear.x, msg.linear.y, msg.linear.z);
  /*
  std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string[] name
float64[] position
float64[] velocity
float64[] effort
*/

  ROS_INFO("A1 Joint State time stamp:   %f", msg.header.stamp);

  ROS_INFO("A1 Joint State Position X:   %f", msg.position[0]);
  ROS_INFO("A1 Joint State Position Y:   %f", msg.position[1]);
  ROS_INFO("A1 Joint State Position Z:   %f", msg.position[2]);

  ROS_INFO("A1 Joint State Velocity X:   %f", msg.velocity[0]);
  ROS_INFO("A1 Joint State Velocity Y:   %f", msg.velocity[1]);
  ROS_INFO("A1 Joint State Velocity Z:   %f", msg.velocity[2]);

}

void recvA2Joint(const sensor_msgs::JointState& msg){

  ROS_INFO("A2 Joint State time stamp:   %f", msg.header.stamp);

  ROS_INFO("A2 Joint State Position X:   %f", msg.position[0]);
  ROS_INFO("A2 Joint State Position Y:   %f", msg.position[1]);
  ROS_INFO("A2 Joint State Position Z:   %f", msg.position[2]);

  ROS_INFO("A2 Joint State Velocity X:   %f", msg.velocity[0]);
  ROS_INFO("A2 Joint State Velocity Y:   %f", msg.velocity[1]);
  ROS_INFO("A2 Joint State Velocity Z:   %f", msg.velocity[2]);

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

  ros::Subscriber sub_a1_joint_states = nh.subscribe("/a1/joint_states", 1, recvA1Joint);
  ros::Subscriber sub_a2_joint_states = nh.subscribe("/a2/joint_states", 1, recvA2Joint);


  ros::spin();
}