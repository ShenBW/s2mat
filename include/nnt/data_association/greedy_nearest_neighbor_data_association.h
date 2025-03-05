#ifndef _GREEDY_NEAREST_NEIGHBOR_DATA_ASSOCIATION_H
#define _GREEDY_NEAREST_NEIGHBOR_DATA_ASSOCIATION_H

#include <nnt/data_association/data_association_interface.h>
#include <nnt/base/defs.h>

namespace nnt
{
class GreedyNearestNeighborDataAssociation : public DataAssociationInterface
{
public:
  /// Initialization method were parameters for data association are read and set accordingly
  virtual void initializeDataAssociation(const ros::NodeHandle& nodeHandle);

  /// Actual data association algorithm which takes tracks and observations and returns Pairings
  virtual Pairings performDataAssociation(Tracks& tracks, const Observations& observations);

  /// typedefs for easier readability
  typedef std::shared_ptr<GreedyNearestNeighborDataAssociation> Ptr;
  typedef std::shared_ptr<const GreedyNearestNeighborDataAssociation> ConstPtr;

private:
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixXXd;
  void calculateCostMatrix(Pairings& pairings, MatrixXXd& costMatrix);
  void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove);
  void removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove);
};

}  // namespace nnt

#endif
