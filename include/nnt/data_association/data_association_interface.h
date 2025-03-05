#ifndef _DATA_ASSOCIATION_INTERFACE_H
#define _DATA_ASSOCIATION_INTERFACE_H

#include <ros/ros.h>
#include <nnt/data/observation.h>
#include <nnt/data/track.h>
#include <nnt/data/pairing.h>
#include <memory>

namespace nnt
{
class DataAssociationInterface
{
public:
  /// Initialization method where parameters for data association are read and set accordingly
  virtual void initializeDataAssociation(const ros::NodeHandle& nodeHandle) = 0;

  /// Actual data association algorithm which takes tracks and observations and returns Pairings
  virtual Pairings performDataAssociation(Tracks& tracks, const Observations& observations) = 0;

  /// typedefs for easier readability
  typedef std::shared_ptr<DataAssociationInterface> Ptr;
  typedef std::shared_ptr<const DataAssociationInterface> ConstPtr;

protected:
  /// Marks a track as matched with the observation in the given pairing
  virtual void markAsMatched(Track::Ptr& track, Pairing::Ptr& pairing)
  {
    ROS_ASSERT(pairing->observation != NULL);

    pairing->validated = true;
    pairing->observation->matched = true;
    track->observation = pairing->observation;
    track->trackStatus = Track::MATCHED;
    track->numberOfTotalMatches++;
    track->numberOfConsecutiveOcclusions = 0;
    track->numberOfConsecutiveMisses = 0;
  }

  ros::NodeHandle m_nodeHandle;
  // ros::NodeHandle m_privateNodeHandle;
};

}  // namespace nnt

#endif
