#ifndef _IMM_HYPOTHESIS_H_
#define _IMM_HYPOTHESIS_H_

#include <vector>
#include <memory>
#include <nnt/base/defs.h>
#include <nnt/data/kalman_filter_state.h>

namespace nnt
{
class IMMState;
class IMMFilter;

class IMMHypothesis : public KalmanFilterState
{
private:
  StateVector m_xMixed;
  StateMatrix m_CMixed;

  double m_probability;
  double m_mixProbability;
  double m_likelihood;
  double m_cNormalizer;

  // FIXME ADAPTION to structure of NNT

public:
  const StateVector& xMixed() const
  {
    return m_xMixed;
  }
  const StateMatrix& CMixed() const
  {
    return m_CMixed;
  }

  double getProbability()
  {
    return m_probability;
  }

  double getMixedProbability()
  {
    return m_mixProbability;
  }

  void setProbability(const double p)
  {
    m_probability = p;
  }

  void setMixedProbability(const double mp)
  {
    m_mixProbability = mp;
  }

  /**
   * Swaps the mixed and regular states.  This avoid an unnecisary copy.
   */
  void setMixedValuesForState()
  {
    m_x = m_xMixed;
    m_C = m_CMixed;
  }

  typedef std::shared_ptr<IMMHypothesis> Ptr;
  typedef std::shared_ptr<const IMMHypothesis> ConstPtr;

  friend class IMMState;
  friend class IMMFilter;
};

typedef std::vector<IMMHypothesis::Ptr> IMMHypotheses;

}  // namespace nnt

#endif /* _IMM_HYPOTHESIS_H_ */
