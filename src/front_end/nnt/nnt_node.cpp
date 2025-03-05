#include <ros/ros.h>
#include <nnt_ros.h>
#include <nnt.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nearest_neighbor_tracker");
  ros::NodeHandle nodeHandle("~");

  // Set up the ROS interface, which also establishes the connection to the parameter server
  nnt::ROSInterface rosInterface(nodeHandle);

  // Now create the tracker and connect to the ROS interface
  nnt::NearestNeighborTracker tracker(nodeHandle);

  rosInterface.connect(&tracker);
  rosInterface.spin();
}
