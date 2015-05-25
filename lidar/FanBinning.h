// $Id: FanBinning.h 1824 2011-01-12 18:41:37Z darko $
#ifndef _lidar_FanBinning_h_
#define _lidar_FanBinning_h_

#include <lidar/Binning.h>


namespace lidar {

  class FanBinning : public Binning {
  public:
    FanBinning(const unsigned int nBins, const Trace& trace);

    FanBinning(const double maxCurrent, const double maxPhotonCount,
               const unsigned int nBins, const Trace& trace);

    unsigned int GetIndex(const Point& p) const;

  private:
    void FindMaxima(const Trace::Type& t);

    double fInvMaxCurrent;
    double fInvMaxPhotonCount;
    unsigned int fNBins;
    double fInvBinAngle;
  };

}

#endif
