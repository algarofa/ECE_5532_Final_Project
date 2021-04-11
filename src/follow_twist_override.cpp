//included packages
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

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

void recvThr(const geometry_msgs::Twist& msg){
  ROS_INFO("rcvThr:     Angular R: %f   P: %f   Y: %f   ", msg.angular.x, msg.angular.y, msg.angular.z);
  ROS_INFO("rcvThr:     Linear  X: %f   Y: %f   Z: %f   ", msg.linear.x, msg.linear.y, msg.linear.z);

  geometry_msgs::Twist vel;

  vel.linear.x = 5.0;
  vel.angular.z = msg.angular.z;

  pub_vel.publish(vel);

  ROS_INFO("newVel:     Linear  X: %f   Y: %f   Z: %f   ", vel.linear.x, vel.linear.y, vel.linear.z);

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

  ros::spin();
}