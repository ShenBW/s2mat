#ifndef _KALMAN_FILTER_STATE_H
#define _KALMAN_FILTER_STATE_H

#include <nnt/data/filter_state.h>

namespace nnt
{
// Forward declarations (for friend declarations)
class EKF;
class IMMFilter;
class IMMState;

/// State and covariances of a Kalman filter.
class KalmanFilterState : public FilterState
{
protected:
  /// Mean (x) of the track state. E.g. x = {x-position, y-position, vx, vy, ax, ay}.
  StateVector m_x;

  /// Covariance matrix (C) of the track state.
  StateMatrix m_C;

private:
  /// Mean (xp) of the predicted track state.
  StateVector m_xp;

  /// Covariance (Cp) of the predicted track state.
  StateMatrix m_Cp;

public:
  // Public getters, as specified in FilterState interface
  virtual const StateVector& x() const
  {
    return m_x;
  }
  virtual const StateMatrix& C() const
  {
    return m_C;
  }
  virtual const StateVector& xp() const
  {
    return m_xp;
  }
  virtual const StateMatrix& Cp() const
  {
    return m_Cp;
  }

  // Public setters, as specified in FilterState interface
  virtual void setX(const StateVector& x)
  {
    m_x = x;
  }
  virtual void setXp(const StateVector& xp)
  {
    m_xp = xp;
  }
  virtual void setC(const StateMatrix& C)
  {
    m_C = C;
  }
  virtual void setCp(const StateMatrix& Cp)
  {
    m_Cp = Cp;
  }

  /// Typedefs for easier readability
  typedef std::shared_ptr<KalmanFilterState> Ptr;
  typedef std::shared_ptr<const KalmanFilterState> ConstPtr;

  /// Return a deep copy of this filter state as a shared pointer
  virtual FilterState::Ptr deepCopy()
  {
    KalmanFilterState* copy = new KalmanFilterState();
    *copy = *this;
    return FilterState::Ptr(copy);
  }

  // Grant write access to certain classes
  friend class EKF;
  friend class IMMFilter;
  friend class IMMState;
};

}  // end of namespace nnt

#endif