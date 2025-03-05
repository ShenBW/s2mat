#ifndef _IMM_FILTER_H_
#define _IMM_FILTER_H_

#include <nnt/filter.h>
#include <nnt/ekf.h>
#include <nnt/data/imm_state.h>
#include <nnt/base/defs.h>

namespace nnt
{
class IMMFilter : public Filter
{
public:
  IMMFilter(const ros::NodeHandle& nodeHandle);

  /// Initialize state and covariance of a new track, based upon the given observation.
  virtual FilterState::Ptr initializeTrackState(Observation::ConstPtr observation,
                                                const VelocityVector& initialVelocity = VelocityVector::Zero());

  /// Predict new track state and covariance by going 'deltatime' into the future and applying the motion model (no new
  /// observation yet).
  virtual void predictTrackState(FilterState::Ptr state, double deltatime);

  /// Update state and covariance of a track using the provided pairing for that track.
  virtual void updateMatchedTrack(FilterState::Ptr state, Pairing::ConstPtr pairing);

  /// Update state and covariance of track which has no corresponding observation (i.e. just using the prediction).
  /// The default implementation just copies the prediction (see predictTrackState()) into the new state.
  virtual void updateOccludedTrack(FilterState::Ptr state);

  /// Set the state transition matrix (which encodes the motion model). Must be called every frame if it is
  /// time-dependent.
  virtual void setTransitionMatrix(const StateVector& x, const double deltaT);

private:
  /// Node handles
  ros::NodeHandle m_nodeHandle;
  // ros::NodeHandle m_privateNodeHandle;

  std::vector<EKF::Ptr> m_kalmanFilters;
  int m_numberModels;

  IMMMatrix m_markovTransitionProbabilities;

  void computeMixingProbabilities(IMMState::Ptr state);
  void doMixing(IMMState::Ptr state);
  void mixPredictions(IMMState::Ptr state);
  void computeMixedMean(IMMState::Ptr state);
  void computeMixedCovariance(IMMState::Ptr state);
  double calcLikelihood(double d, double detS);
  void modeProbabilityUpdate(IMMState::Ptr state);
  void updateStateEstimate(IMMState::Ptr state);
  void updateCurrentHypothesis(IMMState::Ptr state, Track::Ptr track);
};

}  // namespace nnt

#endif /* _IMM_FILTER_H_ */
