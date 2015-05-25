// $Id: PhotonPoint.h 1824 2011-01-12 18:41:37Z darko $
#ifndef _lidar_PhotonPoint_h_
#define _lidar_PhotonPoint_h_
#include <vector>
#include <iostream>


namespace lidar {

  struct PhotonPoint {
    int fIndex;
    double fPhotons;
  };


  inline
  std::ostream&
  operator<<(std::ostream& os, const PhotonPoint& p)
  {
    return os << p.fIndex << ' ' << p.fPhotons;
  }

}


#endif
