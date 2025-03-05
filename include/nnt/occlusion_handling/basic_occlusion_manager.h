#ifndef _BASIC_OCCLUSION_MANAGER_H
#define _BASIC_OCCLUSION_MANAGER_H

#include <nnt/occlusion_handling/occlusion_manager.h>
#include <nnt/data/track.h>

namespace nnt
{
// Simple OcclusionManager that deletes occluded tracks after a fixed number of frames
class BasicOcclusionManager : public OcclusionManager
{
public:
  virtual void initializeOcclusionManager(const ros::NodeHandle& nodehandle);

  virtual Tracks manageOcclusionsBeforeDataAssociation(Tracks& tracks, const ros::Time& time,
                                                       const std::string& trackFrameID);

  virtual void deleteOccludedTracks(Tracks& tracks, const ros::Time& time);

  virtual Pairings occludedTrackAssociation(Tracks tracks, Observations observations, const ros::Time& time)
  {
    return Pairings();
  }

  typedef std::shared_ptr<BasicOcclusionManager> Ptr;
  typedef std::shared_ptr<const BasicOcclusionManager> ConstPtr;

private:
  int m_MAX_OCCLUSIONS_BEFORE_DELETION;
  int m_MAX_OCCLUSIONS_BEFORE_DELETION_OF_MATURE_TRACK;
  int m_TRACK_IS_MATURE_AFTER_TOTAL_NUM_MATCHES;
  double m_viewFieldMinLimitX;
  double m_viewFieldMaxLimitX;
  double m_viewFieldMinLimitY;
  double m_viewFieldMaxLimitY;
};

}  // namespace nnt

#endif
