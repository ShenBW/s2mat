// #include <nnt/base/stl_helpers.h>
#include <nnt/ros/params.h>
#include <nnt/ros/geometry_utils.h>
#include <nnt/imm_filter.h>

#include <tf/tf.h>
#include <ros/ros.h>
#include <Eigen/LU>
#include <limits>
#include <map>

#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>

#define foreach BOOST_FOREACH

namespace nnt
{
IMMFilter::IMMFilter(const ros::NodeHandle& nodeHandle) : m_nodeHandle(nodeHandle)
{
  // TODO Initialization of filter with parameters and prefixes
  m_numberModels = Params::get<int>("number_of_models", 2);
  ROS_INFO_STREAM_NAMED("IMM", "Initializing IMM filter with " << m_numberModels << " models.");

  for (unsigned int i = 0; i < m_numberModels; i++)
  {
    double checksum = 0.0;
    for (unsigned int j = 0; j < m_numberModels; j++)
    {
      std::stringstream markovParamName;
      markovParamName << "markov_trans_prob_" << i << "_" << j;
      m_markovTransitionProbabilities(i, j) = Params::get<double>(markovParamName.str(), 0.0);
      checksum += m_markovTransitionProbabilities(i, j);
      ROS_DEBUG_STREAM("Markov transition probability from " << i << " to " << j << " set to "
                                                             << m_markovTransitionProbabilities(i, j));
    }
    if (std::abs(checksum - 1.0) > 0.01)
      ROS_ERROR_STREAM_NAMED("IMM", "Markov transition probabilities have to sum up to one!");
  }

  for (unsigned int i = 0; i < m_numberModels; i++)
  {
    std::stringstream currrentParameterPrefix;
    currrentParameterPrefix << "IMM" << i << Params::get<std::string>("prefix_model_parameter", "_");
    EKF::Ptr ekf(new EKF(currrentParameterPrefix.str()));
    m_kalmanFilters.push_back(ekf);
  }
}

void IMMFilter::setTransitionMatrix(const StateVector& x, const double deltaT)
{
  ROS_DEBUG_NAMED("IMM", "Setting IMM transitions Matrix to all Kalman filters");
  foreach (EKF::Ptr ekf, m_kalmanFilters)
  {
    ekf->setTransitionMatrix(x, deltaT);
  }
}

FilterState::Ptr IMMFilter::initializeTrackState(Observation::ConstPtr observation,
                                                 const VelocityVector& initialVelocity)
{
  IMMState::Ptr immState(new IMMState);
  ROS_DEBUG_NAMED("IMM", "New IMM state created");

  foreach (EKF::Ptr ekf, m_kalmanFilters)
  {
    // create new hypothesis for each model
    IMMHypothesis::Ptr hypothesis(new IMMHypothesis);
    ekf->initializeTrackState(hypothesis, observation, initialVelocity);
    // initialize probability of hypothesis equally depending on number of models
    hypothesis->m_probability = 1.0 / (double)m_numberModels;
    immState->m_hypotheses.push_back(hypothesis);
    // ROS_INFO_STREAM("Initialize ekf with hypothesis C" << hypothesis->C() << " x " << hypothesis->x());
  }

  // Setting current state to first hypothesis should be therefore the most probable model
  immState->m_currentHypothesis = immState->m_hypotheses.front();
  immState->m_currentHypothesisIdx = 0;
  updateStateEstimate(immState);

  return immState;
}

void IMMFilter::predictTrackState(FilterState::Ptr state, double deltatime)
{
  ROS_DEBUG_NAMED("IMM", "Predict IMM track states!");

  IMMState::Ptr imm = std::dynamic_pointer_cast<IMMState>(state);
  doMixing(imm);

  // Do the traditional kalman filter predictions
  for (int i = 0; i < m_kalmanFilters.size(); i++)
  {
    m_kalmanFilters.at(i)->predictTrackState(imm->m_hypotheses.at(i), deltatime);
    // ROS_WARN_STREAM("Prediction for hypothesis " << i << "with probability  " <<
    // imm->m_hypotheses.at(i)->m_probability << " Cov =  \n" << imm->m_hypotheses.at(i)->C()
    //       << "\n CovP =  \n" << imm->m_hypotheses.at(i)->Cp() << "\n Transition Matrix A \n" <<
    //       m_kalmanFilters.at(i)->m_A);
  }
  mixPredictions(imm);
}

void IMMFilter::mixPredictions(IMMState::Ptr state)
{
  state->m_xp = StateVector::Zero();
  for (int i = 0; i < state->m_hypotheses.size(); i++)
  {
    IMMHypothesis::Ptr hyp = state->m_hypotheses.at(i);
    state->m_xp += hyp->m_xp * hyp->m_probability;
  }

  state->m_Cp = StateMatrix::Zero();
  for (int i = 0; i < state->m_hypotheses.size(); i++)
  {
    IMMHypothesis::Ptr hyp = state->m_hypotheses.at(i);
    StateVector diff = hyp->m_xp - state->m_xp;
    state->m_Cp += (hyp->m_Cp + (diff * diff.transpose())) * hyp->m_probability;
  }
}

void IMMFilter::updateMatchedTrack(FilterState::Ptr state, Pairing::ConstPtr pairingTracker)
{
  // cast to IMM state
  IMMState::Ptr imm = std::dynamic_pointer_cast<IMMState>(state);

  // Do the traditional kalman filter updating
  for (int i = 0; i < m_kalmanFilters.size(); i++)
  {
    // get kalman filter state for current hypothesis
    IMMHypothesis::Ptr hyp = imm->m_hypotheses.at(i);

    // Create a new pairing for later update
    Pairing::Ptr pairing(new Pairing);

    pairing->observation = pairingTracker->observation;
    pairing->validated = pairingTracker->validated;

    // Calculate innovation v and inverse of innovation covariance S
    pairing->v = pairing->observation->z - hyp->zp();
    ObsMatrix S = hyp->H() * hyp->Cp() * hyp->H().transpose() + pairing->observation->R;
    Eigen::FullPivLU<ObsMatrix> lu(S);
    double ln_det_S = log(lu.determinant());

    // Calculate inverse of innovation covariance if possible
    const double MATRIX_LN_EPS = -1e8;

    if (ln_det_S > MATRIX_LN_EPS)
    {
      pairing->Sinv = lu.inverse();
      pairing->d = (pairing->v.transpose() * pairing->Sinv * pairing->v)(0, 0);
      pairing->singular = pairing->d < 0.0;
    }
    else
    {
      pairing->Sinv = ObsMatrix::Constant(OBS_DIM, OBS_DIM, std::numeric_limits<double>::quiet_NaN());
      pairing->d = std::numeric_limits<double>::quiet_NaN();
      pairing->singular = true;

      ROS_WARN_STREAM("Singular pairing encountered!\nTrack  measurement prediction:\n"
                      << state->zp() << "\nTrack covariance prediction:\n"
                      << hyp->Cp() << "\nObservation " << pairing->observation->id << " mean:\n"
                      << pairing->observation->z << "\nObservation covariance:\n"
                      << pairing->observation->R << "\nH:\n"
                      << hyp->H() << "\nS:\n"
                      << S << "\nR:\n"
                      << pairing->observation->R);
    }

    // update kalman filter with pairing for current filter
    ROS_DEBUG_NAMED("IMM", "Updating kalman filter of IMM");
    m_kalmanFilters.at(i)->updateMatchedTrack(hyp, pairing);

    ROS_DEBUG_NAMED("IMM", "Calculating likelihood");

    // save likelihood for hypothesis
    imm->m_hypotheses.at(i)->m_likelihood = calcLikelihood(pairing->d, lu.determinant());
  }

  modeProbabilityUpdate(imm);
  updateStateEstimate(imm);
  updateCurrentHypothesis(imm, pairingTracker->track);
}

double IMMFilter::calcLikelihood(double d, double detS)
{
  // REVIEW showed that this is wrong: NMODELS should not be here and also the square of mahlanobis distance d is shit
  // likelihood = exp(-(d * d) / 2.0) / (sqrt(pow((2 * M_PI), N_MODELS) * detS ));
  // ROS_INFO_STREAM("Exp likelihood " << likelihood);
  // Java implementation does it like this--> we can get rid of the pi factor because its just normalizing and constant
  // for all hypotheses
  double likelihood = exp(-d / 2.0) / sqrt(detS);
  return likelihood;
}

void IMMFilter::updateOccludedTrack(FilterState::Ptr state)
{
  // cast to IMM state
  IMMState::Ptr imm = std::dynamic_pointer_cast<IMMState>(state);

  // Do the traditional kalman filter updating
  for (int i = 0; i < m_kalmanFilters.size(); i++)
  {
    IMMHypothesis::Ptr hyp = imm->getHypothesis(i);
    m_kalmanFilters.at(i)->updateOccludedTrack(hyp);
    // ROS_WARN_STREAM("Occluded update for hypothesis " << i << " Cov =  \n" << hyp->C());
  }

  updateStateEstimate(imm);
  // ROS_ERROR_STREAM("Occluded update mixed Cov= \n" << imm->C());
}

void IMMFilter::doMixing(IMMState::Ptr state)
{
  ROS_DEBUG_NAMED("IMM", "Model mixing");
  computeMixingProbabilities(state);
  computeMixedMean(state);
  computeMixedCovariance(state);
  state->useMixedValues();
}

void IMMFilter::computeMixingProbabilities(IMMState::Ptr state)
{
  // ROS_DEBUG_NAMED("IMM", "Before Loop");
  for (int j = 0; j < state->getHypotheses().size(); j++)
  {
    // FIXME Matrix representation should be possible
    double normalizer = 0.0;
    IMMHypothesis::Ptr hyp = state->getHypothesis(j);

    // Calculate normalizer which is constant for one target hypothesis
    for (int i = 0; i < state->m_hypotheses.size(); i++)
    {
      normalizer += m_markovTransitionProbabilities(i, j) * state->getHypothesis(i)->getProbability();
    }
    hyp->m_cNormalizer = normalizer;

    // Calculate Mixed Probability for each hypothesis
    for (int i = 0; i < state->m_hypotheses.size(); i++)
    {
      state->m_mixingProbabilities(i, j) =
          m_markovTransitionProbabilities(i, j) * state->getHypothesis(i)->getProbability() / hyp->m_cNormalizer;
      ROS_DEBUG_STREAM_NAMED("IMM",
                             "Mixing probability (" << i << ";" << j << ")=" << state->m_mixingProbabilities(i, j));
    }
  }
}

void IMMFilter::computeMixedMean(IMMState::Ptr state)
{
  // Compute mixed initial condition for all hypothesis
  for (int j = 0; j < state->getHypotheses().size(); j++)
  {
    state->getHypothesis(j)->m_xMixed = StateVector::Zero();
    for (int i = 0; i < state->getHypotheses().size(); i++)
    {
      state->getHypothesis(j)->m_xMixed += state->getHypothesis(i)->x() * state->m_mixingProbabilities(i, j);
    }
    ROS_DEBUG_STREAM_NAMED("IMM", "Unmixed Mean x = " << state->getHypothesis(j)->x()
                                                      << " Mixed Mean x = " << state->getHypothesis(j)->xMixed());
  }
}

void IMMFilter::computeMixedCovariance(IMMState::Ptr state)
{
  // Compute mixed covariance taken from book p.456
  // Matrix operations in java are quite strange
  for (int j = 0; j < state->getHypotheses().size(); j++)
  {
    state->getHypothesis(j)->m_CMixed = StateMatrix::Zero();
    for (int i = 0; i < state->getHypotheses().size(); i++)
    {
      StateVector diff = state->getHypothesis(i)->x() - state->getHypothesis(j)->xMixed();
      state->getHypothesis(j)->m_CMixed +=
          state->m_mixingProbabilities(i, j) * (state->getHypothesis(i)->C() + diff * diff.transpose());
    }
    ROS_DEBUG_STREAM_NAMED("IMM", "Unmixed Cov C = " << state->getHypothesis(j)->C()
                                                     << " Mixed Cov Cmixed = " << state->getHypothesis(j)->CMixed());
  }
}

void IMMFilter::updateStateEstimate(IMMState::Ptr state)
{
  state->m_x = StateVector::Zero();
  for (int i = 0; i < state->m_hypotheses.size(); i++)
  {
    IMMHypothesis::Ptr hyp = state->m_hypotheses.at(i);
    state->m_x += hyp->m_x * hyp->m_probability;
  }

  state->m_C = StateMatrix::Zero();
  for (int i = 0; i < state->m_hypotheses.size(); i++)
  {
    IMMHypothesis::Ptr hyp = state->m_hypotheses.at(i);
    StateVector diff = hyp->m_x - state->m_x;
    state->m_C += (hyp->m_C + (diff * diff.transpose())) * hyp->m_probability;
  }
}

void IMMFilter::updateCurrentHypothesis(IMMState::Ptr state, Track::Ptr track)
{
  ROS_DEBUG_NAMED("IMM", "Updating current hypothesis");

  double bestHypothesis = 0.0;
  unsigned int bestHypothesisIdx = 0;
  for (unsigned int i = 0; i < state->m_hypotheses.size(); i++)
  {
    if (state->getHypothesis(i)->m_probability > bestHypothesis)
    {
      bestHypothesis = state->getHypothesis(i)->m_probability;
      bestHypothesisIdx = i;
    }
  }

  if (state->m_currentHypothesis != state->getHypothesis(bestHypothesisIdx))
    ROS_ERROR_STREAM_NAMED("IMM", "Best Hypothesis switched to model " << bestHypothesisIdx);

  state->m_currentHypothesis = state->getHypothesis(bestHypothesisIdx);
  state->m_currentHypothesisIdx = bestHypothesisIdx;
  track->model_idx = bestHypothesisIdx;
  // track->state = state->m_currentHypothesis;
}

void IMMFilter::modeProbabilityUpdate(IMMState::Ptr state)
{
  ROS_DEBUG_NAMED("IMM", "Updating mode probability");

  double normalizer = 0.0;
  for (int i = 0; i < state->m_hypotheses.size(); i++)
  {
    IMMHypothesis::Ptr hyp = state->m_hypotheses.at(i);
    normalizer += hyp->m_likelihood * hyp->m_cNormalizer;
  }
  for (int i = 0; i < state->m_hypotheses.size(); i++)
  {
    IMMHypothesis::Ptr hyp = state->m_hypotheses.at(i);
    hyp->m_probability = hyp->m_likelihood * hyp->m_cNormalizer / normalizer;
    ROS_DEBUG_STREAM_NAMED("IMM", "Mode Probability of hypothesis " << i << "is " << hyp->m_probability);
  }
}

}  // namespace nnt
