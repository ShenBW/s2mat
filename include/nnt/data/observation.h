#ifndef _OBSERVATION_H
#define _OBSERVATION_H

// #include <boost/shared_ptr.hpp>
#include <memory>
#include <vector>
#include <set>
#include <Eigen/Core>

#include <nnt/base/defs.h>

namespace nnt
{
/// Globally unique ID of an observation which is valid over the entire lifetime of the tracker
typedef unsigned int observation_id;

/// An Observation found in a sensor reading.
/// The Observation contains the 2D position z in cartesian space and its covariance R.
struct Observation
{
  /// ID of the Observation.
  observation_id id;

  /// Creation time of the observation in seconds. Usually set from ROS header or logfile timestamp.
  double createdAt;

  /// Observation state vector z.
  ObsVector z;

  /// Covariance R.
  ObsMatrix R;

  double pos_x;
  double pos_y;
  double pos_z;

  double dim_x;
  double dim_y;
  double dim_z;

  /// Confidence in being a tracking target.
  double confidence;

  /// If the observation has been matched against a track.
  bool matched;

  /// The modality or modalities that this observation originates from.
  std::set<std::string> modalities;

  /// Typedefs for easier readability
  typedef std::shared_ptr<Observation> Ptr;
  typedef std::shared_ptr<const Observation> ConstPtr;
};

/// Typedef for easier readability
typedef std::vector<Observation::Ptr> Observations;

}  // end of namespace nnt

#endif