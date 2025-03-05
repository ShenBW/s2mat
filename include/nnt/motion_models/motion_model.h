#ifndef _MOTION_MODEL_H
#define _MOTION_MODEL_H

#include <nnt/base/defs.h>
#include <memory>

namespace nnt
{
/// Abstract class for a generic motion model for an EKF.
class MotionModel
{
public:
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MotionModelMatrix;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> MotionModelVector;

  virtual const MotionModelMatrix& A(const StateVector& x, const double delta_time) = 0;

  virtual const MotionModelMatrix& getProcessNoiseQ(const double delta_time, const double process_noise) = 0;

  virtual const MotionModelMatrix convertToMotionModel(const StateMatrix& matrix) = 0;
  virtual const MotionModelVector convertToMotionModel(const StateVector& vector) = 0;

  virtual const StateMatrix convertToState(const MotionModelMatrix& matrix) = 0;
  virtual const StateVector convertToState(const MotionModelVector& vector) = 0;

  /* --- Other stuff --- */

  /// Typedefs for easier readability
  typedef std::shared_ptr<MotionModel> Ptr;
  typedef std::shared_ptr<const MotionModel> ConstPtr;

  /// Return a deep copy of this filter state as a shared pointer
  virtual Ptr deepCopy() = 0;

protected:
  /// Transition matrix for motion model
  MotionModelMatrix m_A;
  MotionModelMatrix m_Q;
};

}  // namespace nnt

#endif
