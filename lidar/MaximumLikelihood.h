// $Id: MaximumLikelihood.h 1852 2011-02-01 06:49:56Z darko $
#ifndef _lidar_MaximumLikelihood_h_
#define _lidar_MaximumLikelihood_h_
#include <lidar/MeasurementModel.h>
#include <lidar/GlobalMinimizationResult.h>
#include <lidar/Point.h>
#include <lidar/Trace.h>
#include <lidar/PhotonTrace.h>


namespace lidar {

  class MaximumLikelihood {
  public:
    MaximumLikelihood() { }

    MaximumLikelihood(const MeasurementModel& m)
      : fMeasurementModel(m) { }

    MaximumLikelihood(const GlobalMinimizationResult& r)
      : fMeasurementModel(r.fMeasurementModel) { }

    template<class Binning>
    double
    GetDeviance(const Binning& bin, const Trace& trace)
      const
    {
      const Trace::Type& t = trace.fTrace;
      double deviance = 0;
      for (Trace::Type::const_iterator it = t.begin(), end = t.end();
           it != end; ++it) {
        const std::pair<double, double> pd = fMeasurementModel.GetArgMinDeviance(*it);
        deviance += bin.GetWeight(*it) * pd.second;
      }
      return deviance;
    }

    PhotonTrace
    ReconstructPhotons(const Trace& trace)
      const
    {
      const Trace::Type& t = trace.fTrace;
      PhotonTrace ret;
      ret.reserve(t.size());
      for (Trace::Type::const_iterator it = t.begin(), end = t.end();
           it != end; ++it) {
        const PhotonPoint p = { it->fIndex, fMeasurementModel.GetArgMinDeviance(*it).first };
        ret.push_back(p);
      }
      return ret;
    }

    MeasurementModel fMeasurementModel;
  };

}


#endif
