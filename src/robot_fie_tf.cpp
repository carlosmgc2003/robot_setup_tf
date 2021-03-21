#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <numeric>
#include <cmath>



// Numeric Constants
#define PI 3.14159265
// Robot measures
const float MAST_HEIGHT = 1.0f;
const float MAST_POS_X = 0.0f;
const float MAST_POS_Y = 0.0f;

// Global variables
int MEASUREMENT_TIMES = 60;
float MAX_DIST = 0.0f;
std::vector<float> MAX_DISTANCES;
ros::Subscriber lidar;

void lidarMsgCallback(const sensor_msgs::LaserScan::ConstPtr&);

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  // Initial Setup
  ROS_INFO("Initial Setup, do not move robot (Max Distance)");
  lidar = n.subscribe<sensor_msgs::LaserScan>("base_scan", 10, lidarMsgCallback);
  ros::Rate r(60);
  while(MEASUREMENT_TIMES > 0) {
    ros::spinOnce();
    r.sleep();
  }
  auto measures = MAX_DISTANCES.size();
  if(measures != 0) {
    MAX_DIST = std::accumulate(MAX_DISTANCES.begin(), MAX_DISTANCES.end(), 0.0) / measures;
  }

  // Trigonometric Calculations
  ROS_INFO("Average max distance: [%.2f]", MAX_DIST);
  double depression_radians = PI/2 - std::asin(MAST_HEIGHT / MAX_DIST);
  ROS_INFO("Depression degrees: [%.2f]", depression_radians * 180 / PI);
  lidar.shutdown();

  // TF Node Initialization
  ROS_INFO("Ready to tf!");
  tf::TransformBroadcaster broadcaster;
  // Main Loop
  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, depression_radians, 0, 1), tf::Vector3(MAST_POS_X, MAST_POS_Y, MAST_HEIGHT)),
        ros::Time::now(),"base_link", "base_laser"));
    r.sleep();
  } 
  return 0;
}

void lidarMsgCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  float maxDist = msg->range_min;
  for(auto dist: msg->ranges){
    if(dist > maxDist)
      maxDist = dist;
  }
  MAX_DISTANCES.push_back(maxDist);
  MEASUREMENT_TIMES --;
}
