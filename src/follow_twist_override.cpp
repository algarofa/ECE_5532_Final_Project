//included packages
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <gazebo_msgs/ModelStates.h>

//GLOBAL VARIABLES
//For publishing the desired command velocity
ros::Publisher pub_vel;
//global varriable for keeping track of vehicle's distance from one another
double a1_a2_separation;
//Target distance which the cars should be separated by
double sep_target = 25.0;
//Previous following vehicle velocity
double prev_vel;
//vehicle "acceleration"/P gain
double acc_gain = 1.0;

//finds pythagorean distance between two xy points
double cartDistance(double x1, double x2, double y1, double y2)
{
  double xDiff = pow(x1-x2,2);
  double yDiff = pow(y1-y2, 2);

  return sqrt(xDiff+yDiff);
}

//Every time steering command is recieved, also transmit a velocity command
void recvThr(const geometry_msgs::Twist& msg){

  geometry_msgs::Twist vel;

  double new_vel = prev_vel + acc_gain * (a1_a2_separation - sep_target );

  if(new_vel > 30.0)
  {
    new_vel = 30.0;
  }

  if(new_vel < 10.0)
  {
    new_vel = 10.0;
  }

  prev_vel = new_vel;

  vel.linear.x = new_vel;
  vel.angular.z = msg.angular.z;

  pub_vel.publish(vel);
  ROS_INFO("Published Velocity: %f", vel.linear.x);
  ROS_INFO("Distance between cars: %f", a1_a2_separation);

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

  //ROS_INFO("Distance between cars: %f", a1_a2_separation);
}

//main function
int main(int argc, char** argv){
//init section
  ros::init(argc,argv,"follow_twist_override");
  ros::NodeHandle nh;

  ROS_INFO("INTO MAIN");

  ros::Subscriber sub_cmd_vel = nh.subscribe("/a1/cmd_vel", 1, recvThr);

  pub_vel = nh.advertise<geometry_msgs::Twist>("/a1/cmd_vel", 1); //x velocity with float64
  ros::Subscriber sub_gazebo_spy = nh.subscribe("/gazebo/model_states", 1, recvModelStates);

  ros::spin();
}