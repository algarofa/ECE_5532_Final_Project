//http://wiki.ros.org/navigation/Tutorials/RobotSetup/Sensors
//https://docs.ros.org/en/diamondback/api/sensor_msgs/html/LaserScan_8h_source.html
//cartesian coordiantes are generated from the LIDAR array using angle and range and angle info 
//rosmsg show LaserScan
//angle_current = range[i] * angle_min
//need to add costmap, global and local
//global tied to particular TF frame, local tied to base TF frame
//global is static, local is dynamic
//set parameters for costmaps in YAML
//move_base_simple/goal (geometry_msgs/PoseStamped), subscribe to
//cmd_vel, publish to

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h> //message is used to transmit LIDAR data from a driver node to any other node
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

void LaserScan(const sensor_msgs::LaserScanConstPtr& msg){

}

void Velocity_Callback(const geometry_msgs::TwistConstPtr& msg){

}

int main(int argc, char** argv){
  ros::init(argc, argv, "laser_scan");
  ros::NodeHandle nh;
  ros::Publisher pub_lidar = nh.advertise<sensor_msgs::LaserScan>("LaserScan", 50);
  ros::Publisher pub_velocity = nh.advertise<geometry_msgs::Twist>("/a1/cmd_vel", 1);
  //ros::Subscriber sub_lidar = nh.subscribe("/a1/laser_scan", 1, LaserScan_Callback);
  ros::Subscriber sub_velocity = nh.subscribe("/a1/cmd_vel", 1, Velocity_Callback);

  //These lines are already included in LaserScan message, to see more rosmsg show LaserScan
  /*unsigned int num_readings = 100;
  double laser_frequency = 40;
  double ranges[num_readings];
  double intensities[num_readings];

  int count = 0;
  ros::Rate r(1.0);
  while(nh.ok()){
    //generate some fake data for our laser scan
    for(unsigned int i = 0; i < num_readings; ++i){
         ranges[i] = count;
         intensities[i] = 100 + count;
       }
       ros::Time scan_time = ros::Time::now();
   
       //populate the LaserScan message, all properties required
       sensor_msgs::LaserScan scan;
       scan.header.stamp = scan_time;
       scan.header.frame_id = "laser_frame";
       scan.angle_min = -1.57;
       scan.angle_max = 1.57;
       scan.angle_increment = 3.14 / num_readings;
       scan.time_increment = (1 / laser_frequency) / (num_readings);
       scan.range_min = 0.0;
       scan.range_max = 100.0;
   
       scan.ranges.resize(num_readings);
       scan.intensities.resize(num_readings);
       for(unsigned int i = 0; i < num_readings; ++i){
         scan.ranges[i] = ranges[i];
         scan.intensities[i] = intensities[i]; //shiny objects are more intense, opaque objects are less intense, property is the intensity of the reflected laser
       }
   
       scan_pub.publish(scan);
       ++count;
       r.sleep();
     }*/

  ros::spin();
}