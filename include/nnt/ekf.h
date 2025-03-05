#ifndef _EKF_H
#define _EKF_H

#include <nnt/filter.h>
#include <nnt/data/kalman_filter_state.h>
#include <nnt/motion_models/motion_model.h>

namespace nnt
{
class IMMState;
class IMMFilter;

/// Extended Kalman filter for track prediction and update.
/// Assumes the FilterState input argument provided to the functions is of type KalmanFilterState.
class EKF : public Filter
{
public:
  /// Constructor.
  EKF(std::string parameterPrefix = "");

  /// Initialize state and covariance of a new track, based upon the given observation.
  void initializeTrackState(FilterState::Ptr state, Observation::ConstPtr observation,
                            const VelocityVector& initialVelocity = VelocityVector::Zero());

  /// Initialize state and covariance of a new track, based upon the given observation.
  virtual FilterState::Ptr initializeTrackState(Observation::ConstPtr observation,
                                                const VelocityVector& initialVelocity = VelocityVector::Zero());

  /// Predict new track state and covariance by going 'deltatime' into the future and applying the motion model (no new
  /// observation yet).
  virtual void predictTrackState(FilterState::Ptr state, double delta_time);

  /// Update state and covariance of a track using the provided pairing for that track.
  virtual void updateMatchedTrack(FilterState::Ptr state, Pairing::ConstPtr pairing);

  /// Set the state transition matrix (which encodes the motion model). Must be called every frame if it is
  /// time-dependent.
  virtual void setTransitionMatrix(const StateVector& x, const double delta_time);

  virtual void updateOccludedTrack(FilterState::Ptr state);

  friend class IMMState;
  friend class IMMFilter;

  typedef std::shared_ptr<EKF> Ptr;

private:
  /// The state transition matrix A (also sometimes called F)
  StateMatrix m_A;

  /// Initial state covariance at track creation
  StateMatrix m_initialC;

  /// Default additive noise (only used if not using process noise)
  StateMatrix m_defaultQ;

  /// Use process noise? Otherwise use m_defaultQ
  bool m_useProcessNoise;
  double m_processNoiseLevel;

  MotionModel::Ptr m_motionModel;
};

}  // end of namespace nnt

#endif
