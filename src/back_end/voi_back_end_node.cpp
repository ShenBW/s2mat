#include "back_end/voi_back_end.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "back_end");
  ros::NodeHandle nh("~");

  s2mat::VoiBackEnd back_end(nh);

  // ros::spin();
  back_end.run();

  return 0;
}