// $Id: FanBinning.cc 1824 2011-01-12 18:41:37Z darko $
#include <lidar/FanBinning.h>
#include <utl/Accumulator.h>
#include <cmath>

using namespace std;


namespace lidar {

  FanBinning::FanBinning(const unsigned int nBins, const Trace& trace) :
    Binning(),
    fNBins(nBins+1),
    fInvBinAngle(2*nBins/M_PI)
  {
    FindMaxima(trace.fTrace);
    Binning::Bin(trace);
  }


  FanBinning::FanBinning(const double maxCurrent, const double maxPhotonCount,
                         const unsigned int nBins, const Trace& trace) :
    Binning(),
    fInvMaxCurrent(1/(maxCurrent+1)),
    fInvMaxPhotonCount(1/(maxPhotonCount+1)),
    fNBins(nBins+1),
    fInvBinAngle(2*nBins/M_PI)
  {
    Binning::Bin(trace);
  }


  void
  FanBinning::FindMaxima(const Trace::Type& t)
  {
    if (t.empty())
      cerr << "empty trace!" << endl;
    typedef utl::Accumulator::Safe<utl::Accumulator::Max<unsigned int> > Max;
    Max maxCurrent;
    Max maxPhotonCount;
    for (Trace::Type::const_iterator it = t.begin(), end = t.end(); it != end; ++it) {
      maxCurrent(it->fCurrent);
      maxPhotonCount(it->fPhotonCount);
    }
    fInvMaxCurrent = 1. / (maxCurrent.GetMax() + 1);
    fInvMaxPhotonCount = 1. / (maxPhotonCount.GetMax() + 1);
  }


  unsigned int
  FanBinning::GetIndex(const Point& p)
    const
  {
    return (unsigned int)(fInvBinAngle *
                          atan2(p.fPhotonCount*fInvMaxPhotonCount,
                                1 - p.fCurrent*fInvMaxCurrent));
  }

}
