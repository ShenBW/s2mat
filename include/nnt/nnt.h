#ifndef _NNT_H
#define _NNT_H

#include <nnt/base/tracker.h>
#include <nnt/ekf.h>
#include <nnt/imm_filter.h>
#include <nnt/occlusion_handling/basic_occlusion_manager.h>
#include <nnt/data_association/data_association_interface.h>

namespace nnt
{
/// A nearest-neighbor standard filter (NNSF) tracker. Processes observations (usually at a fixed interval, depending on
/// sensor refresh rate), and outputs a new set of tracked objects.
class NearestNeighborTracker : public Tracker
{
public:
  /// Constructor.
  explicit NearestNeighborTracker(ros::NodeHandle& nodeHandle);

  virtual ~NearestNeighborTracker();

  /// Process a single tracking time-step using the new set of observations. Returns the currently tracked targets.
  virtual const Tracks& processCycle(double currentTime, const Observations& newObservations);

  /// Returns the number of the current tracking cycle, starts at 0 for the first set of observations
  virtual unsigned long int getCurrentCycleNo()
  {
    return m_cycleCounter;
  }

private:
  /// Resets all necessary variables to start a new tracking cycle.
  void beginCycle(double currentTime);

  /// Predict track movement based upon a motion model (e.g. constant velocity), not yet taking any new observations
  /// into account.
  void predictTrackStates();

  /// After predicting new track states, predict how a measurement corresponding to that state would look like.
  void predictMeasurements();

  /// Update the Extended Kalman filter for each track using the matched observation, if any, or just copy the predicted
  /// state into the posterior.
  void updateKalmanFilter(const Pairings& pairings);

  // Avoid creating duplicate tracks if there is a close-by track that has recently been matched
  bool checkForClosebyExistingTrack(const Observation::Ptr& observation);

  /// Creates a new track from the given observation.
  Track::Ptr createTrack(const Observation::Ptr& observation);

  /// Create new tracks from unmatched observations.
  void initNewTracksFromObservations(const Observations& newObservations);

  /// Delete tracks which have not seen any matching observation in a while.
  void deleteObsoleteTracks();

  /// Delete duplicate tracks
  void deleteDuplicateTracks();

  /// Increment cycle number and finish the current tracking cycle.
  void endCycle();

  /// Node handles
  ros::NodeHandle m_nodeHandle;

  /// FrameID where tracker operates
  std::string m_frameID;

  /// The tracks that are currently being maintained/tracked.
  Tracks m_tracks;

  /// The extended Kalman filter used for track prediction and update
  Filter::Ptr m_filter;

  /// Global track ID counter
  track_id m_trackIdCounter;

  /// The current cycle which we are in, starts at 0 with first set of observations
  unsigned long int m_cycleCounter;

  /// Time when the current tracking cycle started, in seconds; and seconds passed since last cycle started
  double m_cycleTime, m_deltaTime;

  // Occlusion handling and track deletion logic.
  OcclusionManager::Ptr m_occlusionManager;

  // The currently used data association.
  DataAssociationInterface::Ptr m_dataAssociation;
};

}  // end of namespace nnt

#endif