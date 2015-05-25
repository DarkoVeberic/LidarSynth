// $Id: TrivialBinning.h 1823 2011-01-12 15:43:48Z darko $
#ifndef _lidar_TrivialBinning_h_
#define _lidar_TrivialBinning_h_

#include <lidar/Trace.h>


namespace lidar {

  class TrivialBinning {
  public:
    TrivialBinning(const Trace& trace) : fN(trace.fTrace.size()) { }

    double GetWeight(const Point& /*p*/) const { return 1; }

    double GetNorm() const { return fN; }

  private:
    unsigned int fN;
  };

}


#endif
