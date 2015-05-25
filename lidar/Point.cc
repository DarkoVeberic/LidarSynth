#include <lidar/Point.h>

using namespace std;


namespace lidar {

  ostream&
  operator<<(ostream& os, const Point& p)
  {
    return os << p.fIndex << ' ' << p.fCurrent << ' ' << p.fPhotonCount;
  }


  istream&
  operator>>(istream& is, Point& p)
  {
    return is >> p.fCurrent >> std::ws >> p.fPhotonCount;
  }

}
