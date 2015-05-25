// $Id: CornerBinning.h 1823 2011-01-12 15:43:48Z darko $
#ifndef _lidar_CornerBinning_h_
#define _lidar_CornerBinning_h_

#include <lidar/Binning.h>
#include <lidar/Trace.h>
#include <utl/LinearFunction.h>
#include <vector>


namespace lidar {

  class CornerBinning : public Binning {
  public:
    CornerBinning(const Trace& trace, const double nBins) :
      Binning(),
      fNBins(nBins)
    {
      SetupIndexing(trace);
      Binning::Bin(trace);
    }

    unsigned int GetIndex(const Point& p) const;

  private:
    void SetupIndexing(const Trace& trace);

    double fNBins;
    utl::LinearFunction fX;
    utl::LinearFunction fY;
  };

}


#endif
