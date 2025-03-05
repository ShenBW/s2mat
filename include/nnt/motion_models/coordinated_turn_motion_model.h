#ifndef _COORDINATED_TURN_MOTION_MODEL_H
#define _COORDINATED_TURN_MOTION_MODEL_H

#include <nnt/motion_models/motion_model.h>

namespace nnt
{
class CoordinatedTurnMotionModel : public MotionModel
{
public:
  CoordinatedTurnMotionModel()
  {
    m_A = MotionModelMatrix::Zero(DIM, DIM);
    m_Q = MotionModelMatrix::Zero(DIM, DIM);
  }

  virtual const MotionModelMatrix& A(const StateVector& x, const double delta_time)
  {
    m_A = MotionModelMatrix::Identity(DIM, DIM);
    /// get readable access to current state
    double omegak = x(STATE_OMEGA_IDX);
    double xk = x(STATE_X_IDX);
    double vxk = x(STATE_VX_IDX);
    double yk = x(STATE_Y_IDX);
    double vyk = x(STATE_VY_IDX);

    if (fabs(omegak) > 1e-6)
    {
      /// calculate often needed values
      double sinOmegaT = sin(omegak * delta_time);
      double cosOmegaT = cos(omegak * delta_time);

      double omegak_2 = omegak * omegak;

      /// calculate Jacobian
      m_A(STATE_X_IDX, STATE_VX_IDX) = sinOmegaT / omegak;
      m_A(STATE_X_IDX, STATE_VY_IDX) = -1 * (1 - cosOmegaT) / omegak;
      m_A(STATE_VX_IDX, STATE_VX_IDX) = cosOmegaT;
      m_A(STATE_VX_IDX, STATE_VY_IDX) = -sinOmegaT;
      m_A(STATE_Y_IDX, STATE_VX_IDX) = (1 - cosOmegaT) / omegak;
      m_A(STATE_Y_IDX, STATE_VY_IDX) = sinOmegaT / omegak;
      m_A(STATE_VY_IDX, STATE_VX_IDX) = sinOmegaT;
      m_A(STATE_VY_IDX, STATE_VY_IDX) = cosOmegaT;
      /// Calculate partial derivatives with respect to omega
      m_A(STATE_X_IDX, STATE_OMEGA_IDX) = (cosOmegaT * delta_time * vxk) / omegak - (sinOmegaT * vxk) / omegak_2 -
                                          (sinOmegaT * delta_time * vyk) / omegak - ((-1 + cosOmegaT) * vyk) / omegak_2;
      m_A(STATE_Y_IDX, STATE_OMEGA_IDX) = (sinOmegaT * delta_time * vxk) / omegak - ((1 - cosOmegaT) * vxk) / omegak_2 +
                                          (cosOmegaT * delta_time * vyk) / omegak - (sinOmegaT * vyk) / omegak_2;
      m_A(STATE_VX_IDX, STATE_OMEGA_IDX) = (-sinOmegaT * delta_time * vxk) - (cosOmegaT * delta_time * vyk);
      m_A(STATE_VY_IDX, STATE_OMEGA_IDX) = (cosOmegaT * delta_time * vxk) - (sinOmegaT * delta_time * vyk);
    }
    else
    {
      /// calculate Jacobian limiting form
      m_A(STATE_X_IDX, STATE_VX_IDX) = delta_time;
      m_A(STATE_X_IDX, STATE_OMEGA_IDX) = -0.5 * (delta_time * delta_time) * vyk;
      m_A(STATE_VX_IDX, STATE_OMEGA_IDX) = -delta_time * vyk;
      m_A(STATE_Y_IDX, STATE_VY_IDX) = delta_time;
      m_A(STATE_Y_IDX, STATE_OMEGA_IDX) = 0.5 * (delta_time * delta_time) * vxk;
      m_A(STATE_VY_IDX, STATE_OMEGA_IDX) = delta_time * vxk;
    }
    return m_A;
  }

  virtual const MotionModelMatrix& getProcessNoiseQ(const double delta_time, const double process_noise)
  {
    // get variance for turn rate omega in rad/s (default: 10 deg/s)
    double sigmaOmega = 0.17453292519;

    // Process noise according to "Modern Tracking Systems" page 210
    m_Q = MotionModelMatrix::Zero(DIM, DIM);
    /*m_Q(0, 0) = (delta_time * delta_time * delta_time * delta_time) * process_noise / 4.0;
    m_Q(1, 1) = m_Q(0, 0);
    m_Q(0, 2) = 0.5 * (delta_time * delta_time* delta_time) * process_noise;
    m_Q(1, 3) = m_Q(0, 2);
    m_Q(2, 0) = m_Q(0, 2);
    m_Q(3, 1) = m_Q(0, 2);
    m_Q(2, 2) = (delta_time*delta_time) * process_noise;
    m_Q(3, 3) = m_Q(2, 2);
    m_Q(4, 4) = (sigmaOmega * sigmaOmega) * (delta_time * delta_time);*/

    m_Q(0, 0) = (delta_time * delta_time * delta_time) * process_noise / 3.0;
    m_Q(1, 1) = m_Q(0, 0);
    m_Q(0, 2) = 0.5 * (delta_time * delta_time) * process_noise;
    m_Q(1, 3) = m_Q(0, 2);
    m_Q(2, 0) = m_Q(0, 2);
    m_Q(3, 1) = m_Q(0, 2);
    m_Q(2, 2) = delta_time * process_noise;
    m_Q(3, 3) = m_Q(2, 2);
    // m_Q(4, 4) = (sigmaOmega * sigmaOmega) * (delta_time * delta_time);
    m_Q(4, 4) = sigmaOmega * delta_time;

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
  typedef std::shared_ptr<CoordinatedTurnMotionModel> Ptr;
  typedef std::shared_ptr<const CoordinatedTurnMotionModel> ConstPtr;

  /// Return a deep copy of this motion model as a shared pointer
  virtual MotionModel::Ptr deepCopy()
  {
    CoordinatedTurnMotionModel* copy = new CoordinatedTurnMotionModel();
    *copy = *this;
    return MotionModel::Ptr(copy);
  }

private:
  const static int DIM = 5;
};

}  // namespace nnt

#endif
