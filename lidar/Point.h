#ifndef _lidar_Point_h_
#define _lidar_Point_h_

#include <iostream>


namespace lidar {

  class Point {
  public:
    int fIndex;
    unsigned int fCurrent;
    unsigned int fPhotonCount;
  };


  std::ostream& operator<<(std::ostream& os, const Point& p);


  std::istream& operator>>(std::istream& is, Point& p);

}


#endif
