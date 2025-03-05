#ifndef _WIENER_PROCESS_ACCELERATION_MOTION_MODEL_H
#define _WIENER_PROCESS_ACCELERATION_MOTION_MODEL_H

#include <nnt/motion_models/motion_model.h>

namespace nnt
{
class WienerProcessAccelerationModel : public MotionModel
{
public:
  WienerProcessAccelerationModel()
  {
    m_A = MotionModelMatrix::Zero(DIM, DIM);
    m_Q = MotionModelMatrix::Zero(DIM, DIM);
  }

  virtual const MotionModelMatrix& A(const StateVector& x, const double delta_time)
  {
    m_A = MotionModelMatrix::Identity(DIM, DIM);
    m_A(IDX_X, IDX_VX) = delta_time;
    m_A(IDX_Y, IDX_VY) = delta_time;
    m_A(IDX_X, IDX_AX) = 0.5 * (delta_time * delta_time);
    m_A(IDX_Y, IDX_AY) = m_A(IDX_X, IDX_AX);
    m_A(IDX_VX, IDX_AX) = delta_time;
    m_A(IDX_VY, IDX_AY) = delta_time;
    return m_A;
  }

  virtual const MotionModelMatrix& getProcessNoiseQ(const double delta_time, const double process_noise)
  {
    m_Q = MotionModelMatrix::Zero(DIM, DIM);
    // According to p.276 in Estimation wit Application to Tracking and Navigation
    m_Q(IDX_X, IDX_X) = (delta_time * delta_time * delta_time * delta_time * delta_time) * process_noise / 20.0;
    m_Q(IDX_Y, IDX_Y) = m_Q(IDX_X, IDX_X);
    m_Q(IDX_VX, IDX_X) = (delta_time * delta_time * delta_time * delta_time) * process_noise / 8.0;
    m_Q(IDX_X, IDX_VX) = m_Q(IDX_VX, IDX_X);
    m_Q(IDX_Y, IDX_VY) = m_Q(IDX_VX, IDX_X);
    m_Q(IDX_VY, IDX_Y) = m_Q(IDX_VX, IDX_X);
    m_Q(IDX_X, IDX_AX) = (delta_time * delta_time * delta_time) * process_noise / 6.0;
    m_Q(IDX_AX, IDX_X) = m_Q(IDX_X, IDX_AX);
    m_Q(IDX_Y, IDX_Y) = m_Q(IDX_X, IDX_AX);
    m_Q(IDX_AY, IDX_AY) = m_Q(IDX_X, IDX_AX);
    m_Q(IDX_VX, IDX_VX) = (delta_time * delta_time * delta_time) * process_noise / 3.0;
    m_Q(IDX_VY, IDX_VY) = m_Q(IDX_VX, IDX_VX);
    m_Q(IDX_X, IDX_VX) = (delta_time * delta_time) * process_noise / 2.0;
    m_Q(IDX_VX, IDX_X) = m_Q(IDX_X, IDX_VX);
    m_Q(IDX_VY, IDX_Y) = m_Q(IDX_X, IDX_VX);
    m_Q(IDX_Y, IDX_VY) = m_Q(IDX_X, IDX_VX);
    m_Q(IDX_AX, IDX_AX) = delta_time * process_noise;
    m_Q(IDX_AY, IDX_AY) = m_Q(IDX_AX, IDX_AX);

    return m_Q;
  }

  virtual const MotionModelMatrix convertToMotionModel(const StateMatrix& matrix)
  {
    MotionModelMatrix model = matrix;
    removeColumn(model, STATE_OMEGA_IDX);
    removeRow(model, STATE_OMEGA_IDX);
    // ROS_INFO_STREAM(matrix << "\nConverted to motionmodel matrix:\n" << model);
    return model;
  }

  virtual const MotionModelVector convertToMotionModel(const StateVector& vector)
  {
    MotionModelVector model = MotionModelVector::Zero(DIM);
    model.head(STATE_VY_IDX) = vector.head(STATE_VY_IDX);
    model.tail(2) = vector.tail(2);
    // ROS_INFO_STREAM(vector << "\nConverted to motionmodel vector:\n" << model);
    return model;
  }

  virtual const StateMatrix convertToState(const MotionModelMatrix& matrix)
  {
    StateMatrix state = StateMatrix::Identity();
    state.block<4, 4>(0, 0) = matrix.block<4, 4>(0, 0);
    state.block<2, 4>(STATE_AX_IDX, STATE_X_IDX) = matrix.block<2, 4>(IDX_AX, IDX_X);
    state.block<4, 2>(STATE_X_IDX, STATE_AX_IDX) = matrix.block<4, 2>(IDX_X, IDX_AX);
    state(STATE_AX_IDX, STATE_AX_IDX) = matrix(IDX_AX, IDX_AX);
    state(STATE_AY_IDX, STATE_AY_IDX) = matrix(IDX_AY, IDX_AY);
    // ROS_INFO_STREAM(matrix << "\nConverted to state matrix:\n" << state);
    return state;
  }
  virtual const StateVector convertToState(const MotionModelVector& vector)
  {
    StateVector state = StateVector::Zero();
    state.head(4) = vector.head(4);
    state.tail(2) = vector.tail(2);
    // ROS_INFO_STREAM(vector << "\nConverted to state vector:\n" << state);
    return state;
  }

  /* --- Other stuff --- */

  /// Typedefs for easier readability
  typedef std::shared_ptr<WienerProcessAccelerationModel> Ptr;
  typedef std::shared_ptr<const WienerProcessAccelerationModel> ConstPtr;

  /// Return a deep copy of this motion model as a shared pointer
  virtual MotionModel::Ptr deepCopy()
  {
    WienerProcessAccelerationModel* copy = new WienerProcessAccelerationModel();
    *copy = *this;
    return MotionModel::Ptr(copy);
  }

private:
  const static int DIM = 6;
  const static unsigned int IDX_X = 0;
  const static unsigned int IDX_Y = 1;
  const static unsigned int IDX_VX = 2;
  const static unsigned int IDX_VY = 3;
  const static unsigned int IDX_AX = 4;
  const static unsigned int IDX_AY = 5;

  void removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove)
  {
    unsigned int numRows = matrix.rows() - 1;
    unsigned int numCols = matrix.cols();

    if (rowToRemove < numRows)
      matrix.block(rowToRemove, 0, numRows - rowToRemove, numCols) =
          matrix.block(rowToRemove + 1, 0, numRows - rowToRemove, numCols);

    matrix.conservativeResize(numRows, numCols);
  }

  void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove)
  {
    unsigned int numRows = matrix.rows();
    unsigned int numCols = matrix.cols() - 1;

    if (colToRemove < numCols)
      matrix.block(0, colToRemove, numRows, numCols - colToRemove) =
          matrix.block(0, colToRemove + 1, numRows, numCols - colToRemove);

    matrix.conservativeResize(numRows, numCols);
  }
};

}  // namespace nnt

#endif
