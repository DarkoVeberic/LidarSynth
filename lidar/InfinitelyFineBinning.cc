// $Id: InfinitelyFineBinning.cc 1824 2011-01-12 18:41:37Z darko $
#include <lidar/InfinitelyFineBinning.h>
#include <utl/Accumulator.h>

using namespace std;


namespace lidar {

  double
  InfinitelyFineBinning::GetNorm()
    const
  {
    double sum = 0;
    for (MUD::const_iterator it = fMap.begin(), end = fMap.end();
         it != end; ++it)
      sum += it->second;
    return sum;
  }


  double
  InfinitelyFineBinning::GetNorm(const Trace& trace)
    const
  {
    const Trace::Type& t = trace.fTrace;
    double sum = 0;
    for (Trace::Type::const_iterator it = t.begin(), end = t.end();
         it != end; ++it)
      sum += GetWeight(*it);
    return sum;
  }


  void
  InfinitelyFineBinning::FindMaxPhotonCount1(const Trace& trace)
  {
    using namespace utl::Accumulator;
    fMaxPhotonCount1 =
      for_each(trace.PhotonCountBegin(), trace.PhotonCountEnd(),
               Safe<Max<unsigned int> >()).GetMax() + 1;
  }


  void
  InfinitelyFineBinning::Map(const Trace::Type& t)
  {
    typedef std::map<unsigned int, unsigned int> MUU;
    MUU m;
    for (Trace::Type::const_iterator it = t.begin(), end = t.end();
         it != end; ++it) {
      const unsigned int i = GetIndex(*it);
      const MUU::iterator found = m.find(i);
      if (found == m.end())
        m.insert(make_pair(i, 1U));
      else
        ++found->second;
    }
    double sum = 0;
    for (MUU::const_iterator it = m.begin(), end = m.end(); it != end; ++it) {
      const double invCount = 1. / it->second;
      fMap.insert(make_pair(it->first, invCount));
      sum += invCount;
    }
    sum = t.size() / sum;
    for (MUD::iterator it = fMap.begin(), end = fMap.end(); it != end; ++it)
      it->second *= sum;
  }

}
