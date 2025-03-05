#ifndef _TRACK_H
#define _TRACK_H

// #include <boost/shared_ptr.hpp>
#include <memory>
#include <boost/circular_buffer.hpp>
#include <vector>

#include <nnt/data/observation.h>
#include <nnt/data/kalman_filter_state.h>
#include <nnt/base/defs.h>

namespace nnt
{
/// Globally unique ID of a track which is valid over the entire lifetime of the tracker
typedef unsigned int track_id;
typedef unsigned int model_index;

/// A Track is the representation of an object, observed and tracked with an Extended Kalman Filter.
struct Track
{
  /// State of a track
  enum TrackStatus
  {
    NEW,
    MATCHED,
    MISSED,
    OCCLUDED,
    DELETED
  };

  /// State of the track.
  TrackStatus trackStatus;

  /// Globally unique track ID
  track_id id;

  /// Model index for IMM Models
  model_index model_idx;

  /// Initializing or matching Observation. NULL for OCCLUDED or DELETED tracks.
  Observation::Ptr observation;

  /// Creation time of the track in seconds.
  double createdAt;

  /// Number of matches during the life-cycle of the Track.
  unsigned int numberOfTotalMatches;

  /// Number of consecutive occlusions.
  unsigned int numberOfConsecutiveOcclusions;

  /// Number of consecutive misses.
  unsigned int numberOfConsecutiveMisses;

  /// Number of consecutive weak matches (with a recovered observation, e.g. by actively searching the laser scan)
  unsigned int numberOfConsecutiveWeakMatches;

  /// Detection probability
  double detectionProbability;

  /// Filter state (e.g. from a Kalman filter), including current and predicted track position
  FilterState::Ptr state;

  /// History of filter states for debugging.
  typedef boost::circular_buffer<FilterState::Ptr> FilterStateHistory;
  FilterStateHistory stateHistory;

  /// Typedefs for easier readability
  typedef std::shared_ptr<Track> Ptr;
  typedef std::shared_ptr<const Track> ConstPtr;
};

/// Typedef for easier readability
typedef std::vector<Track::Ptr> Tracks;

}  // end of namespace nnt

#endif
