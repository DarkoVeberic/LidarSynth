// $Id: FilterSaturation.cc 1824 2011-01-12 18:41:37Z darko $
#include <lidar/FilterSaturation.h>

using namespace std;


namespace lidar {

  Trace
  FilterSaturation(const Trace& trace,
                   const unsigned int maxADC)
  {
    Trace rTrace(trace.fNShots);
    const unsigned int maxCurrent = trace.fNShots * maxADC;

    const Trace::Type& t = trace.fTrace;
    for (Trace::Type::const_iterator it = t.begin(), end = t.end();
         it != end; ++it)
      if (it->fCurrent < maxCurrent) 
        rTrace.fTrace.push_back(*it);

    return rTrace;
  }

}
