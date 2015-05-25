// $Id: CornerBinning.cc 1824 2011-01-12 18:41:37Z darko $
#include <lidar/CornerBinning.h>
#include <utl/Accumulator.h>

using namespace std;


namespace lidar {

  unsigned int
  CornerBinning::GetIndex(const Point& p)
    const
  {
    const double x = fX(p.fCurrent);
    const double y = fY(p.fPhotonCount);
    return (x >= y) ? 2 * int(y) : 2 * int(x) + 1;
  }


  void
  CornerBinning::SetupIndexing(const Trace& trace)
  {
    using namespace utl::Accumulator;
    const Trace::Type& t = trace.fTrace;
    Safe<MinMax<unsigned int> > ca;
    Safe<Max<unsigned int> > pca;
    for (Trace::Type::const_iterator it = t.begin(), end = t.end();
         it != end; ++it) {
      ca(it->fCurrent);
      pca(it->fPhotonCount);
    }
    const unsigned int cLim = (ca.GetMin() + ca.GetMax()) / 2;
    const unsigned int pcLim = pca.GetMax() / 2;
    Mean cAvg, pcAvg;
    for (Trace::Type::const_iterator it = t.begin(), end = t.end();
         it != end; ++it) {
      if (it->fCurrent > cLim)
        pcAvg(it->fPhotonCount);
      if (it->fPhotonCount < pcLim)
        cAvg(it->fCurrent);
    }
    if (fNBins <= 0)
      fNBins = 1;
    const double d = fNBins / (ca.GetMax() - cAvg.GetMean());
    fX = utl::LinearFunction(-d, d*ca.GetMax());
    fY = utl::LinearFunction(fNBins/pcAvg.GetMean());
  }

}
