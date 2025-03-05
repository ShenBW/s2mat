#include <ros/ros.h>
#include "map_merger/map_merger.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_merge");
  ros::NodeHandle nh("~");

  smat::MapMerger map_merger(nh);

  map_merger.run();

  // ros::spin();

  return 0;
}