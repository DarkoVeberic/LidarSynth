// $Id: TotalDeviance.h 1849 2011-01-31 06:39:54Z darko $
#ifndef _lidar_TotalDeviance_h_
#define _lidar_TotalDeviance_h_
#include <lidar/MeasurementModel.h>
#include <lidar/MaximumLikelihood.h>
#include <lidar/Trace.h>

namespace lidar {

  template<class Binning>
  double
  TotalDeviance(const MeasurementModel& mm, const Binning& bin, const Trace& trace)
  {
    return MaximumLikelihood(mm).GetDeviance(bin, trace);
  }


  inline
  double
  TotalDeviance(const MeasurementModel& mm, const Trace& trace)
  {
    return MaximumLikelihood(mm).GetDeviance(TrivialBinning(trace), trace);
  }

}


#endif
