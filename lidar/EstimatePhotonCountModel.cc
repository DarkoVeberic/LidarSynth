// $Id: EstimatePhotonCountModel.cc 1849 2011-01-31 06:39:54Z darko $
#include <lidar/EstimatePhotonCountModel.h>
#include <utl/Accumulator.h>

using namespace std;


namespace lidar {

  PhotonCountModel
  EstimatePhotonCountModel(const Trace& trace, const double upperFitFraction)
  {
    using namespace utl::Accumulator;
    typedef Safe<MinMax<unsigned int> > MinMax;
    const MinMax minMax = for_each(trace.CurrentBegin(), trace.CurrentEnd(), MinMax());
    if (!minMax || minMax.GetMin() == minMax.GetMax()) {
      const PhotonCountModel pcm(0);
      return pcm;
    }
    const unsigned int minFit =
      minMax.GetMax() - upperFitFraction * (minMax.GetMax() - minMax.GetMin());
    Mean avg;
    for (Trace::Type::const_iterator it = trace.fTrace.begin(),
         end = trace.fTrace.end(); it != end; ++it)
      if (it->fCurrent >= minFit)
        avg(it->fPhotonCount);
    return PhotonCountModel(1 / avg.GetMean());
  }

}
