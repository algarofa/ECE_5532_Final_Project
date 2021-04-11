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
}

//main function
int main(int argc, char** argv){
//init section
  ros::init(argc,argv,"gps_sim_proj");
  ros::NodeHandle nh;

  pub_vel = nh.advertise<geometry_msgs::Twist>("/override/cmd_vel", 1); //x velocity with float64
}