#ifndef _TRACKER_H
#define _TRACKER_H

#include <nnt/data/observation.h>
#include <nnt/data/track.h>

namespace nnt
{
/// Abstract tracker base class.
class Tracker
{
public:
  /// Destructor.
  virtual ~Tracker(){};

  /// Process a single tracking time-step using the new set of observations. Returns the currently tracked targets.
  virtual const Tracks& processCycle(double currentTime, const Observations& newObservations) = 0;

  /// Returns the number of the current tracking cycle, starts at 0 for the first set of observations
  virtual unsigned long int getCurrentCycleNo() = 0;
};

}  // end of namespace nnt

#endif  // _TRACKER_H