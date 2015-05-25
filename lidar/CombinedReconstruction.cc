// $Id: CombinedReconstruction.cc 1849 2011-01-31 06:39:54Z darko $
#include <lidar/CombinedReconstruction.h>


namespace lidar {

  CombinedReconstructionTrace
  CombinedReconstruction(const Trace& trace, const PhotonTrace& pTrace,
                         const MeasurementModel& mm)
  {
    CombinedReconstructionTrace ret;
    const Trace::Type& t = trace.fTrace;
    const unsigned int n = t.size();
    if (n != pTrace.size())
      return ret;
    ret.reserve(t.size());
    for (unsigned int i = 0; i < n; ++i) {
      const Point& m = t[i];
      const double p = pTrace[i].fPhotons;
      const CombinedReconstructionPoint crp = {
        m.fIndex,
        m.fCurrent,
        m.fPhotonCount,
        p,
        mm.fCurrentModel.EstimateCurrent(p),
        mm.fPhotonCountModel.EstimateCount(p),
        mm.fCurrentModel.EstimatePhoton(m.fCurrent),
        mm.fPhotonCountModel.EstimatePhoton(m.fPhotonCount)
      };
      ret.push_back(crp);
    }
    return ret;
  }

}
