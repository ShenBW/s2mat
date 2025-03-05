#ifndef _FILTER_H
#define _FILTER_H

#include <nnt/data/filter_state.h>
#include <nnt/data/observation.h>
#include <nnt/data/pairing.h>

namespace nnt
{
/// Generic filter interface (e.g. for a Kalman filter / Particle filter / IMM filter)
class Filter
{
public:
  /// Initialize state and covariance of a new track, based upon the given observation.
  virtual FilterState::Ptr initializeTrackState(Observation::ConstPtr observation,
                                                const VelocityVector& initial_velocity = VelocityVector::Zero()) = 0;

  /// Predict new track state and covariance by going 'deltatime' into the future and applying the motion model (no new
  /// observation yet).
  virtual void predictTrackState(FilterState::Ptr state, double delta_time) = 0;

  /// Update state and covariance of a track using the provided pairing for that track.
  virtual void updateMatchedTrack(FilterState::Ptr state, Pairing::ConstPtr pairing) = 0;

  /// Set the state transition matrix (which encodes the motion model). Must be called every frame if it is
  /// time-dependent.
  virtual void setTransitionMatrix(const StateVector& x, const double delta_time) = 0;

  typedef std::shared_ptr<Filter> Ptr;
  typedef std::shared_ptr<const Filter> ConstPtr;
};

}  // namespace nnt

#endif