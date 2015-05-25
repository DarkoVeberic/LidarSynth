// $Id: EstimateCurrentModel.cc 1849 2011-01-31 06:39:54Z darko $
#include <lidar/EstimateCurrentModel.h>
#include <utl/Accumulator.h>
#include <utl/Identity.h>
#include <vector>

using namespace std;
using namespace utl;


namespace lidar {

  template<class Transform>
  CurrentModel
  Estimate(const Trace& trace, const Transform& trans, const double lowerFitFraction)
  {
    using namespace Accumulator;
    typedef Safe<Max<unsigned int> > SMax;
    const SMax max = for_each(trace.PhotonCountBegin(), trace.PhotonCountEnd(), SMax());
    if (!max || max.GetMax() == 0) {
      const CurrentModel cm(0, 0, 0);
      return cm;
    }
    const unsigned int maxFit = lowerFitFraction * max.GetMax();
    LinearFitWithVar fit;
    for (Trace::Type::const_iterator it = trace.fTrace.begin(),
         end = trace.fTrace.end(); it != end; ++it)
      if (it->fPhotonCount < maxFit)
        fit(trans(it->fPhotonCount), it->fCurrent);
    const pair<double, double> ab = fit.GetCoefficients();
    const CurrentModel cm(ab.second, ab.first, fit.GetVar());
    return cm;
  }


  CurrentModel
  EstimateCurrentModel(const Trace& trace, const double lowerFitFraction)
  {
    return Estimate<>(trace, Identity<unsigned int>(), lowerFitFraction);
  }


  class DeadTimeCorrection {
  public:
    DeadTimeCorrection(const PhotonCountModel& pcm)
      : fPCModel(pcm) { }

    double operator()(const unsigned int c) const
    { return fPCModel.EstimatePhoton(c); }

  private:
    const PhotonCountModel& fPCModel;
  };


  CurrentModel
  EstimateCurrentModel(const Trace& trace, const PhotonCountModel& pcm,
                       const double lowerFitFraction)
  {
    return Estimate(trace, DeadTimeCorrection(pcm), lowerFitFraction);
  }

}
