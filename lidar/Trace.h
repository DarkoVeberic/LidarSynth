#ifndef _lidar_Trace_h_
#define _lidar_Trace_h_

#include <lidar/Point.h>
#include <vector>
#include <boost/iterator/transform_iterator.hpp>


namespace lidar {

  class Trace {
  private:
    struct InternalCurrentFunctor {
      unsigned int operator()(const Point& p) const
      { return p.fCurrent; }
    };

    struct InternalPhotonCountFunctor {
      unsigned int operator()(const Point& p) const
      { return p.fPhotonCount; }
    };

  public:
    typedef std::vector<Point> Type;

    Trace(const unsigned int nShots) : fNShots(nShots) { }

    typedef
    boost::transform_iterator<
      InternalCurrentFunctor,
      Type::const_iterator,
      unsigned int
    > CurrentIterator;

    typedef
    boost::transform_iterator<
      InternalPhotonCountFunctor,
      Type::const_iterator,
      unsigned int>
    PhotonCountIterator;

    CurrentIterator CurrentBegin() const
    { return CurrentIterator(fTrace.begin()); }
    CurrentIterator CurrentEnd() const
    { return CurrentIterator(fTrace.end()); }

    PhotonCountIterator PhotonCountBegin() const
    { return PhotonCountIterator(fTrace.begin()); }
    PhotonCountIterator PhotonCountEnd() const
    { return PhotonCountIterator(fTrace.end()); }

    unsigned int fNShots;
    Type fTrace;
  };

}


#endif
