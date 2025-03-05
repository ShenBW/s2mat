#include <nnt/ros/params.h>

namespace nnt
{
Params* Params::s_instance = NULL;

Params::Params(ros::NodeHandle& privateNodeHandle) : m_privateNodeHandle(privateNodeHandle)
{
  assert(!s_instance);
  s_instance = this;
}

}  // end of namespace nnt