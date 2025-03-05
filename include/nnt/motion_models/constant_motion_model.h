#ifndef _CONSTANT_MOTION_MODEL_H
#define _CONSTANT_MOTION_MODEL_H

#include <nnt/motion_models/motion_model.h>

namespace nnt
{
class ConstantMotionModel : public MotionModel
{
public:
  ConstantMotionModel()
  {
    m_A = MotionModelMatrix::Zero(DIM, DIM);
    m_Q = MotionModelMatrix::Zero(DIM, DIM);
  }

  virtual const MotionModelMatrix& A(const StateVector& x, const double delta_time)
  {
    m_A = MotionModelMatrix::Identity(DIM, DIM);
    m_A(STATE_X_IDX, STATE_VX_IDX) = delta_time;
    m_A(STATE_Y_IDX, STATE_VY_IDX) = delta_time;
    return m_A;
  }

  virtual const MotionModelMatrix& getProcessNoiseQ(const double delta_time, const double process_noise)
  {
    m_Q = MotionModelMatrix::Zero(DIM, DIM);
    m_Q(0, 0) = (delta_time * delta_time * delta_time) * process_noise / 3.0;
    m_Q(1, 1) = m_Q(0, 0);
    m_Q(0, 2) = 0.5 * (delta_time * delta_time) * process_noise;
    m_Q(1, 3) = m_Q(0, 2);
    m_Q(2, 0) = m_Q(0, 2);
    m_Q(3, 1) = m_Q(0, 2);
    m_Q(2, 2) = delta_time * process_noise;
    m_Q(3, 3) = m_Q(2, 2);
    return m_Q;
  }

  virtual const MotionModelMatrix convertToMotionModel(const StateMatrix& matrix)
  {
    MotionModelMatrix model = matrix.block<DIM, DIM>(0, 0);
    return model;
  }

  virtual const MotionModelVector convertToMotionModel(const StateVector& vector)
  {
    MotionModelVector model = vector.head(DIM);
    return model;
  }

  virtual const StateMatrix convertToState(const MotionModelMatrix& matrix)
  {
    StateMatrix state = StateMatrix::Identity();
    state.block<DIM, DIM>(0, 0) = matrix.block<DIM, DIM>(0, 0);
    return state;
  }
  virtual const StateVector convertToState(const MotionModelVector& vector)
  {
    StateVector state = StateVector::Zero();
    state.head(DIM) = vector.head(DIM);
    return state;
  }

  /* --- Other stuff --- */

  /// Typedefs for easier readability
  typedef std::shared_ptr<ConstantMotionModel> Ptr;
  typedef std::shared_ptr<const ConstantMotionModel> ConstPtr;

  /// Return a deep copy of this motion model as a shared pointer
  virtual MotionModel::Ptr deepCopy()
  {
    ConstantMotionModel* copy = new ConstantMotionModel();
    *copy = *this;
    return MotionModel::Ptr(copy);
  }

private:
  const static int DIM = 4;
};

}  // namespace nnt

#endif
