#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <numeric>
#include <algorithm>
#include <cmath>



// Numeric Constants
#define PI 3.14159265
// Robot measures
const float MAST_HEIGHT = 1.22;
const float MAST_POS_X = 0.0;
const float MAST_POS_Y = 0.0;
const int RANGES_VECTOR = 1080;
// One fifth of total vector
const int PARTS = 65;
const int ELEMENTS = RANGES_VECTOR / PARTS;
const int START_FRONT_RANGES = (RANGES_VECTOR / 2 ) - (ELEMENTS / 2);
const int END_FRONT_RANGES = (RANGES_VECTOR / 2) + (ELEMENTS / 2);

// Global variables
int MEASUREMENT_TIMES = 120;
float MAX_DIST = 0.0f;
std::vector<float> MAX_DISTANCES;
ros::Subscriber lidar;

void lidarMsgCallback(const sensor_msgs::LaserScan::ConstPtr&);

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  // Initial Setup
  ROS_INFO("Initial Setup, do not move robot (Max Distance)");
  lidar = n.subscribe<sensor_msgs::LaserScan>("scan", 10, lidarMsgCallback);
  ros::Rate r(60);
  while(MEASUREMENT_TIMES > 0) {
    ros::spinOnce();
    r.sleep();
  }
  ROS_INFO("measures: %d", MAX_DISTANCES.size());
  auto measures = MAX_DISTANCES.size();
  if(measures != 0) {
    std::sort(MAX_DISTANCES.begin(), MAX_DISTANCES.end(), std::greater<float>());
    MAX_DIST = MAX_DISTANCES[MAX_DISTANCES.size() / 2];
  }

  // Trigonometric Calculations
  ROS_INFO("Average max distance: [%.2f]", MAX_DIST);
  double depression_radians = std::asin(MAST_HEIGHT / MAX_DIST);
  ROS_INFO("Depression degrees: [%.2f]", depression_radians * 180 / PI);
  ROS_INFO("Depression radians: [%.2f]", depression_radians);
  lidar.shutdown();

  // TF Node Initialization
  ROS_INFO("Ready to tf!");
  tf::TransformBroadcaster broadcaster;
  // Main Loop
  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(depression_radians, 0, 0), tf::Vector3(MAST_POS_X, MAST_POS_Y, MAST_HEIGHT)),
        ros::Time::now(),"base_link", "laser"));
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(MAST_POS_X, MAST_POS_Y, MAST_HEIGHT - 0.2)),
        ros::Time::now(),"base_link", "zed_camera_center"));
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0), tf::Vector3(MAST_POS_X, MAST_POS_Y, MAST_HEIGHT - 0.17)),
        ros::Time::now(),"base_link", "imu_link"));
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0), tf::Vector3(MAST_POS_X, MAST_POS_Y,0.30)),
        ros::Time::now(),"base_link", "base_stabilized"));
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0), tf::Vector3(MAST_POS_X, MAST_POS_Y,0.0)),
        ros::Time::now(),"base_link", "base_footprint"));


    r.sleep();
  } 
  return 0;
}

void lidarMsgCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  float maxDist = msg->range_min;
  auto cont = 0;
  for(auto i = START_FRONT_RANGES; i < END_FRONT_RANGES; i ++) {
    cont ++;
    float dist = msg->ranges[i];
    if(dist > maxDist)
      maxDist = dist;
  }
  ROS_INFO("Start: %d, End: %d, Cont: %d, minDist: %.2f",START_FRONT_RANGES, END_FRONT_RANGES, cont, maxDist);
  MAX_DISTANCES.push_back(maxDist);
  MEASUREMENT_TIMES --;
}
