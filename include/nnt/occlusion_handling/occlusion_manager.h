#ifndef _OCCLUSION_MANAGER_H
#define _OCCLUSION_MANAGER_H

#include <nnt/data/track.h>
#include <nnt/data/pairing.h>
#include <ros/ros.h>
#include <memory>
// #include <nnt/base/stl_helpers.h>

namespace nnt
{
/// Generic occlusion manager interface for deleting occluded tracks after a certain time
class OcclusionManager
{
public:
  virtual void initializeOcclusionManager(const ros::NodeHandle& nodehandle) = 0;

  virtual Tracks manageOcclusionsBeforeDataAssociation(Tracks& tracks, const ros::Time& time,
                                                       const std::string& trackFrameID) = 0;

  virtual void deleteOccludedTracks(Tracks& tracks, const ros::Time& time) = 0;

  virtual Pairings occludedTrackAssociation(Tracks tracks, Observations observations, const ros::Time& time) = 0;

  void setFrameIDofTracker(const std::string& frameID)
  {
    m_frameIDTracker = frameID;
  }

  typedef std::shared_ptr<OcclusionManager> Ptr;
  typedef std::shared_ptr<const OcclusionManager> ConstPtr;

protected:
  /// Frame ID where tracker operates
  std::string m_frameIDTracker;
};

}  // namespace nnt

#endif
