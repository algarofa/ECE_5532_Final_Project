//included packages
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelStates.h>
#include "../PID/cpp/PID.h"
#include <sensor_msgs/Image.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

using namespace cv;
using namespace std;

//https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html


//GLOBAL VARIABLES
//For publishing the desired command velocity
ros::Publisher pub_vel;
//global varriable for keeping track of vehicle's distance from one another
double a1_a2_separation = 0;
//Target distance which the cars should be separated by
double sep_target = 60;
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
double P = 1.0;
double I = 0.0;
double D = 0.0;
//Height of vehicle in pixels
int pix_height = 0;

ros::Publisher pub_thresh;

//publishes velocity and steering command messages
void cmdVel(double v)
{
  geometry_msgs::Twist vel;

  vel.linear.x = v;
  vel.angular.z = cmd_turn;

  pub_vel.publish(vel);
  ROS_INFO("Published Velocity: %f", vel.linear.x);
  ROS_INFO("Distance between cars: %f", a1_a2_separation);
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

  //pid_source = sep_target - a1_a2_separation;

  //ROS_INFO("recvModelStates: %f", a1_a2_separation);
}

//init PID controller
PIDController<double> vel_PID_controller(P, I, D, pidDoubleSource, pidDoubleOutput);

//refreshes PID
void PIDTimerCallback(const ros::TimerEvent& event){
  vel_PID_controller.tick();/*
  ROS_INFO("GPS distance: %f", a1_a2_separation);
  ROS_INFO("PID ticked");
  ROS_INFO("PID Target: %f", vel_PID_controller.getTarget());
  ROS_INFO("PID error: %f", vel_PID_controller.getError());
  ROS_INFO("PID output: %f", vel_PID_controller.getOutput());
  ROS_INFO("PID feedback: %f", vel_PID_controller.getFeedback());*/
}

void recvImage(const sensor_msgs::ImageConstPtr& msg)
{
  int x_0 = 0;
  int y_0 = 0;
  int x_f = msg->width;
  int y_f = msg->width - 450; //cropping of vertical

  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat raw_img = cv_ptr->image;

  cv::imshow("Raw Image", raw_img);
  cv::waitKey(1);

  std::vector<cv::Mat> split_images;
  cv::split(raw_img, split_images);

  cv::Mat blue_image = split_images[1];
  //cv::Mat green_image = split_images[1];
  //cv::Mat red_image = split_images[2];

  cv::Mat croppedImage = blue_image(cv::Rect(x_0, y_0, x_f, y_f));       
  cv::imshow("Cropped Image", croppedImage);

  cv::imshow("Blue Image", blue_image);
  cv::waitKey(1);

  cv::Mat thres_img;
  cv::threshold(croppedImage, thres_img, 2, 255, cv::THRESH_BINARY);

  cv::imshow("Thres Image", thres_img);
  cv::waitKey(1);

  cv::Mat erode_img;
  cv::erode(thres_img, erode_img, cv::Mat::ones(20, 20, CV_8U));

  cv::imshow("Erode Image", erode_img);
  cv::waitKey(1);

  cv::Mat dilate_img;
  cv::dilate(erode_img, dilate_img, cv::Mat::ones(20, 20, CV_8U));

  cv::imshow("Dilate Image", dilate_img);
  cv::waitKey(1);

  cv::Mat canny_output;
  Canny(dilate_img, canny_output, 64, 192);
  vector<vector<Point> > contours;
  findContours( canny_output, contours, RETR_TREE, CHAIN_APPROX_SIMPLE );

  vector<vector<Point> > contours_poly( contours.size() );
  vector<Rect> boundRect( contours.size() );
  
  RNG rng(12345);
  int max_height = 0;

  for( size_t i = 0; i < contours.size(); i++ )
  {
      approxPolyDP( contours[i], contours_poly[i], 3, true );
      boundRect[i] = boundingRect( contours_poly[i] );
      //minEnclosingCircle( contours_poly[i], centers[i], radius[i] );
  }
  Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
  for( size_t i = 0; i< contours.size(); i++ )
  {
      Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
      //drawContours( drawing, contours_poly, (int)i, color );
      rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2 );
      if(boundRect[i].height>max_height)
      {
        max_height = boundRect[i].height;
      }
      //circle( drawing, centers[i], (int)radius[i], color, 2 );
  }

  pix_height = max_height;
  
  //ROS_INFO("Number of contours: %d", contours.size());
  //ROS_INFO("max height: %d", pix_height);
  imshow( "Contoured Image", drawing );

  pid_source = pix_height - sep_target;

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

  //create troubleshooting GUI windows
  cv::namedWindow("Raw Image", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("Blue Image", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("Cropped Image", CV_WINDOW_AUTOSIZE);
  //cv::namedWindow("Red Image", CV_WINDOW_AUTOSIZE);
  //cv::namedWindow("Green Image", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("Thres Image", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("Erode Image", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("Dilate Image", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("Contoured Image", CV_WINDOW_AUTOSIZE);

  ros::Subscriber sub_image = nh.subscribe("/a1/front_camera/image_raw", 1, &recvImage);

  ros::spin();
}