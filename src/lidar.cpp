//http://wiki.ros.org/navigation/Tutorials/RobotSetup/Sensors
//Cartesian coordiantes are generated from the LIDAR array using angle and range and angle info 
//rosmsg show LaserScan
//currentangle = range index * angle_min

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h> //message is used to transmit LIDAR data from a driver node to any other node

int main(int argc, char** argv){
  ros::init(argc, argv, "laser_scan");
  ros::NodeHandle nh;
  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 50);

  unsigned int num_readings = 100;
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
     }

  ros::spin();
}