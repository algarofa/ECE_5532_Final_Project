#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>
#include <ugv_course_libs/gps_conv.h>

UTMCoords ref_coords1;
UTMCoords ref_coords2;
tf::Vector3 relative_position1;
tf::Vector3 relative_position2;
nav_msgs::Path gps_path;
ros::Publisher path_pub;

void timerCallback(const ros::TimerEvent& event){
  geometry_msgs::PoseStamped current_pose1;
  current_pose1.pose.position.x = relative_position1.x();
  current_pose1.pose.position.y = relative_position1.y();

  geometry_msgs::PoseStamped current_pose2;
  current_pose2.pose.position.x = relative_position2.x();
  current_pose2.pose.position.y = relative_position2.y();
  
  gps_path.poses.push_back(current_pose1);
  gps_path.header.frame_id="world";
  gps_path.header.stamp = event.current_real;
  path_pub.publish(gps_path);
}

void recvFix1(const sensor_msgs::NavSatFixConstPtr& msg){
  UTMCoords current_coords(*msg);

  relative_position1 = current_coords - ref_coords1;

  ROS_INFO("Current UTM: (%f, %f)", current_coords.getX(), current_coords.getY());
  ROS_INFO("Relative Position: (%f, %f)", relative_position1.x(), relative_position1.y());
  
}

void recvFix2(const sensor_msgs::NavSatFixConstPtr& msg){
  UTMCoords current_coords(*msg);

  relative_position2 = current_coords - ref_coords2;

  ROS_INFO("Current UTM: (%f, %f)", current_coords.getX(), current_coords.getY());
  ROS_INFO("Relative Position: (%f, %f)", relative_position2.x(), relative_position2.y());
  
}

int main(int argc, char** argv){
  ros::init(argc,argv,"gps_example");
  ros::NodeHandle nh;
  ros::Subscriber gps_sub1 = nh.subscribe("/a1/gps/fix",1,recvFix1);
  ros::Subscriber gps_sub2 = nh.subscribe("/a2/gps/fix",1,recvFix2);
  ros::Timer timer = nh.createTimer(ros::Duration(0.5), timerCallback);
  path_pub = nh.advertise<nav_msgs::Path>("gps_path",1);

  double ref_lat1, ref_lon1;
  double ref_lat2, ref_lon2;

  nh.getParam("/a1/gps/ref_lat",ref_lat1);
  nh.getParam("/a2/gps/ref_lon",ref_lon1);
  
  nh.getParam("/a1/gps/ref_lat",ref_lat2);
  nh.getParam("/a2/gps/ref_lon",ref_lon2);

  LatLon ref_coords_lat_lon1(ref_lat1, ref_lon1, 0);
  ref_coords1 = UTMCoords(ref_coords_lat_lon1);

  LatLon ref_coords_lat_lon2(ref_lat2, ref_lon2, 0);
  ref_coords2 = UTMCoords(ref_coords_lat_lon2);

  double central_meridian = ref_coords1.getCentralMeridian();
  double central_meridian = ref_coords2.getCentralMeridian();

  ROS_INFO("Central Meridian of the Reference Cooridinate: %f", central_meridian);
  
  ros::spin();
}