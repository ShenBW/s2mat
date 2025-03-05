#include "front_end/rimg_front_end.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "front_end");
  ros::NodeHandle nh("~");

  smat::RimgFrontEnd front_end(nh);

  front_end.subscribePointcloud();

  front_end.run();

  // ros::spin();
  return 0;
}