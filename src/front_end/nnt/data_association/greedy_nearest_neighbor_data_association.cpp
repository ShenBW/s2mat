

#include <nnt/data_association/greedy_nearest_neighbor_data_association.h>

#include <nnt/base/defs.h>
// #include <nnt/base/stl_helpers.h>
#include <nnt/ros/params.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <Eigen/LU>
#include <limits>
#include <map>

#define VERY_BIG_COST BIG_COST + 1111

namespace nnt
{
void GreedyNearestNeighborDataAssociation::initializeDataAssociation(const ros::NodeHandle& nodeHandle)
{
  m_nodeHandle = nodeHandle;
}

Pairings GreedyNearestNeighborDataAssociation::performDataAssociation(Tracks& tracks, const Observations& observations)
{
  ROS_DEBUG("Performing data association");

  //
  // Step 0: Make sure observations and tracks are in a good state
  //

  foreach (Observation::Ptr observation, observations)
  {
    observation->matched = false;
  }

  foreach (Track::Ptr track, tracks)
  {
    track->observation.reset();

    // Incremented counters, if matched they will be set to 0 anyway
    if (track->trackStatus == Track::OCCLUDED)
    {
      // Track is occluded
      track->trackStatus = Track::OCCLUDED;
      track->numberOfConsecutiveOcclusions++;
    }
    else
    {
      // Track is missed
      track->trackStatus = Track::MISSED;
      track->numberOfConsecutiveMisses++;
    }
  }

  Pairings compatiblePairings;

  //
  // Step 1:  go through all possible associations and store them in a array
  //          we fill a cost matrix with all possible associations
  //

  if (tracks.size() > 0 && observations.size() > 0)
  {
    const double MATRIX_LN_EPS = -1e8;
    const double MAX_GATING_DISTANCE = Params::get<double>("max_gating_distance", 1.0);
    const double MAX_VOL_DIFF = Params::get<double>("max_vol_diff", 0.5);

    Eigen::MatrixXd costMatrix = Eigen::MatrixXd::Constant(tracks.size(), observations.size(), BIG_COST);
    Pairing pairingArray[tracks.size()][observations.size()];
    for (size_t t = 0; t < tracks.size(); t++)
    {
      Track::Ptr track = tracks.at(t);

      Pairings trackPairings;
      for (size_t ob = 0; ob < observations.size(); ob++)
      {
        Observation::Ptr observation = observations.at(ob);

        bool vol_match = true;
        if (track->observation)
        {
          double track_vol = track->observation->dim_x * track->observation->dim_y * track->observation->dim_z;
          double obs_vol = observation->dim_x * observation->dim_y * observation->dim_z;
          vol_match = abs(obs_vol - track_vol) < MAX_VOL_DIFF;
        }

        // Create a new pairing
        Pairing& pairingRef(pairingArray[t][ob]);

        // Calculate innovation v and inverse of innovation covariance S
        ObsVector v = observation->z - track->state->zp();

        // do simple gating here to save expensive operations
        if (v.norm() < MAX_GATING_DISTANCE && vol_match)
        {
          pairingRef.v = v;
          pairingRef.track = track;
          pairingRef.observation = observation;
          pairingRef.validated = false;

          ObsMatrix S = track->state->H() * track->state->Cp() * track->state->H().transpose() + observation->R;
          Eigen::FullPivLU<ObsMatrix> lu(S);
          double ln_det_S = 0;
          ln_det_S = log(lu.determinant());
          // Calculate inverse of innovation covariance if possible
          if (ln_det_S > MATRIX_LN_EPS)
          {
            pairingRef.Sinv = lu.inverse();
            pairingRef.d = (pairingRef.v.transpose() * pairingRef.Sinv * pairingRef.v)(0, 0);
            pairingRef.singular = pairingRef.d < 0.0;
          }
          else
          {
            pairingRef.Sinv = ObsMatrix::Constant(OBS_DIM, OBS_DIM, std::numeric_limits<double>::quiet_NaN());
            pairingRef.d = std::numeric_limits<double>::quiet_NaN();
            pairingRef.singular = true;

            ROS_WARN_STREAM("Singular pairing encountered!\nTrack "
                            << track->id << " measurement prediction:\n"
                            << track->state->zp() << "\nTrack covariance prediction:\n"
                            << track->state->Cp() << "\nObservation " << observation->id << " mean:\n"
                            << observation->z << "\nObservation covariance:\n"
                            << observation->R);
          }

          // Perform gating
          if ((!pairingRef.singular) && (pairingRef.d < CHI2INV_99[OBS_DIM]))
          {
            // if((!pairingRef.singular)) {
            // Store in list of compatible pairings
            if (Params::get<bool>("use_correlation_log", false))
              costMatrix(t, ob) = (pairingRef.d + ln_det_S) / track->detectionProbability;
            else
              costMatrix(t, ob) = (pairingRef.d) / track->detectionProbability;
            // ROS_INFO_STREAM("Track id: " << track->id << "Current cost = " << costMatrix(t, ob) << " d " <<
            // pairingRef.d << " lnS " << ln_det_S);
          }
        }
      }
    }

    //
    // Step 2: find minimum in cost Matrix until there is no valid pairing anymore
    //
    Eigen::VectorXd observationBIG = Eigen::VectorXd::Constant(observations.size(), VERY_BIG_COST);
    Eigen::VectorXd tracksBIG = Eigen::VectorXd::Constant(tracks.size(), VERY_BIG_COST);

    double currentCost = 0;
    int i, j;
    do
    {
      currentCost = costMatrix.minCoeff(&i, &j);
      Track::Ptr track = tracks.at(i);
      if (currentCost < BIG_COST)
      {
        compatiblePairings.push_back(std::make_shared<Pairing>(pairingArray[i][j]));
        // ROS_DEBUG_STREAM("Assigning track " << i << "(" << track->id << ")"<< " to observation " << j);
        markAsMatched(track, compatiblePairings.back());
        // Setting found association to BIG_COST in cost matrix
        // ROS_INFO_STREAM("Cost Matrix " << costMatrix);
        // ROS_INFO_STREAM("tracks " << costMatrix.row(i));
        // ROS_INFO_STREAM("obs " << costMatrix.col(i));
        // ROS_INFO_STREAM("tracksBIG " << tracksBIG);
        // ROS_INFO_STREAM("observationBIG " << observationBIG);
        costMatrix.row(i) = observationBIG;
        costMatrix.col(j) = tracksBIG;
      }
    } while (currentCost < BIG_COST);
  }

  ROS_DEBUG("%zu compatible pairings have been found for %zu existing tracks and %zu new observations!",
            compatiblePairings.size(), tracks.size(), observations.size());
  return compatiblePairings;
}

void GreedyNearestNeighborDataAssociation::removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove)
{
  unsigned int numRows = matrix.rows() - 1;
  unsigned int numCols = matrix.cols();

  if (rowToRemove < numRows)
    matrix.block(rowToRemove, 0, numRows - rowToRemove, numCols) =
        matrix.block(rowToRemove + 1, 0, numRows - rowToRemove, numCols);

  matrix.conservativeResize(numRows, numCols);
}

void GreedyNearestNeighborDataAssociation::removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove)
{
  unsigned int numRows = matrix.rows();
  unsigned int numCols = matrix.cols() - 1;

  if (colToRemove < numCols)
    matrix.block(0, colToRemove, numRows, numCols - colToRemove) =
        matrix.block(0, colToRemove + 1, numRows, numCols - colToRemove);

  matrix.conservativeResize(numRows, numCols);
}

}  // namespace nnt
