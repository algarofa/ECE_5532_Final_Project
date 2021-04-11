//included packages
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

//global variables
ros::Publisher pub_vel;

//periodically sends a speed command
void speedTimerCallback(const ros::TimerEvent& event)
{
  geometry_msgs::Twist vel;

  vel.linear.x = 5.0;
  //ack_vel.angular.z = psi_doECE_5532_Final_Projectt;

  pub_vel.publish(vel);

  ROS_INFO("Published velocity");
}

//main function
int main(int argc, char** argv){
//init section
  ros::init(argc,argv,"follow_twist_override");
  ros::NodeHandle nh;

  ROS_INFO("INTO MAIN");

  ros::Timer timer = nh.createTimer(ros::Duration(1.0), speedTimerCallback);

  pub_vel = nh.advertise<geometry_msgs::Twist>("/override/cmd_vel", 1); //x velocity with float64
  //ros::Subscriber sub_twist = nh.subscribe("/audibot/throttle_cmd", 1, recvThr);

  ros::spin();
}