// $Id: ShiftPhotonCount.cc 1827 2011-01-14 15:19:05Z darko $
#include <lidar/ShiftPhotonCount.h>
#include <cmath>

using namespace std;


namespace lidar {

  // offset relative to fixed current
  // indexing remains equal to the original current
  Trace
  ShiftPhotonCount(const Trace& trace, const int offset)
  {
    if (!offset)
      return trace;
    Trace res(trace.fNShots);
    const Trace::Type& t = trace.fTrace;
    const unsigned int absOffset = abs(offset);
    if (absOffset >= t.size())
      return res;
    Trace::Type& rt = res.fTrace;
    rt.reserve(t.size() - absOffset);
    if (offset > 0)
      for (unsigned int i = offset, j = 0, n = t.size(); i < n; ++i, ++j) {
        const Point d = { t[i].fIndex, t[i].fCurrent, t[j].fPhotonCount };
        rt.push_back(d);
      }
    else
      for (unsigned int i = 0, j = -offset, n = t.size(); j < n; ++i, ++j) {
        const Point d = { t[i].fIndex, t[i].fCurrent, t[j].fPhotonCount };
        rt.push_back(d);
      }
    return res;
  }

}
