// $Id: InfinitelyFineBinning.h 1824 2011-01-12 18:41:37Z darko $
#ifndef _lidar_InfinitelyFineBinning_h_
#define _lidar_InfinitelyFineBinning_h_

#include <lidar/Trace.h>
#include <vector>
#include <map>


namespace lidar {

  class InfinitelyFineBinning {
  public:
    InfinitelyFineBinning(const Trace& trace)
    { FindMaxPhotonCount1(trace); Map(trace.fTrace); }

    double GetWeight(const Point& p) const
    { return fMap.find(GetIndex(p))->second; }

    unsigned int GetNNonemptyBins() const { return fMap.size(); }

    double GetNorm() const;

    double GetNorm(const Trace& trace) const;

  private:
    void FindMaxPhotonCount1(const Trace& trace);

    unsigned int GetIndex(const Point& p) const
    { return p.fPhotonCount + fMaxPhotonCount1 * p.fCurrent; }


    void Map(const Trace::Type& t);

    unsigned int fMaxPhotonCount1;
    typedef std::map<unsigned int, double> MUD;
    MUD fMap;
  };

}

#endif
