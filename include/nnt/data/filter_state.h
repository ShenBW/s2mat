#ifndef _FILTER_STATE_H
#define _FILTER_STATE_H

#include <Eigen/Core>
#include <memory>

#include <nnt/base/defs.h>

namespace nnt
{
/// Abstract class for a generic track filter's state (e.g. Kalman filter / Particle filter / IMM filter state).
class FilterState
{
public:
  /* --- Public getters. These assume the filter state and prediction can be represented as a mixture of Gaussians. ---
   */

  /// Mean (x) of the track state. E.g. x = [ x-position, y-position, vx, vy ].
  virtual const StateVector& x() const = 0;

  /// Covariance matrix (C) of the track state.
  virtual const StateMatrix& C() const = 0;

  /// Mean (xp) of the predicted track state.
  virtual const StateVector& xp() const = 0;

  /// Covariance (Cp) of the predicted track state.
  virtual const StateMatrix& Cp() const = 0;

  /// Track measurement prediction (zp).
  virtual const ObsVector& zp() const
  {
    return m_zp;
  }

  /// Track measurement Jacobian (H).
  virtual const ObsStateMatrix& H() const
  {
    return m_H;
  }

  /// Mean (x) of the track state. E.g. x = [ x-position, y-position, vx, vy ].
  virtual void setX(const StateVector& x) = 0;

  /// Mean (x) of the track state. E.g. x = [ x-position, y-position, vx, vy ].
  virtual void setXp(const StateVector& xp) = 0;

  /// Mean (x) of the track state. E.g. x = [ x-position, y-position, vx, vy ].
  virtual void setC(const StateMatrix& C) = 0;

  /// Mean (x) of the track state. E.g. x = [ x-position, y-position, vx, vy ].
  virtual void setCp(const StateMatrix& Cp) = 0;

  /* --- Other stuff --- */

  /// Typedefs for easier readability
  typedef std::shared_ptr<FilterState> Ptr;
  typedef std::shared_ptr<const FilterState> ConstPtr;

  /// Return a deep copy of this filter state as a shared pointer
  virtual Ptr deepCopy() = 0;

  /// Recompute the measurement prediction zp based upon the current xp and the given new Jacobian H.
  virtual void updateMeasurementPrediction(ObsStateMatrix& newH)
  {
    m_H = newH;
    m_zp = m_H * xp();
  }

protected:
  ObsVector m_zp;      /// Track measurement prediction
  ObsStateMatrix m_H;  /// Track measurement Jacobian
};

}  // namespace nnt

#endif