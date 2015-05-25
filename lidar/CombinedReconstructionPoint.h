// $Id: CombinedReconstructionPoint.h 1859 2011-05-31 14:29:42Z darko $
#ifndef _lidar_CombinedReconstructionPoint_h_
#define _lidar_CombinedReconstructionPoint_h_
#include <lidar/Point.h>
#include <lidar/PhotonPoint.h>
#include <iostream>


namespace lidar {

  struct CombinedReconstructionPoint {
    int fIndex;
    unsigned int fMeasuredCurrent;
    unsigned int fMeasuredPhotonCount;
    double fReconstructedPhotons;
    double fCurrentFromPhotons;
    double fPhotonCountFromPhotons;
    double fPhotonsFromMeasuredCurrent;
    double fPhotonsFromMeasuredPhotonCount;
  };


  inline
  std::ostream&
  operator<<(std::ostream& os, const CombinedReconstructionPoint& p)
  {
    return os << p.fIndex << ' '
              << p.fMeasuredCurrent << ' '
              << p.fMeasuredPhotonCount << ' '
              << p.fReconstructedPhotons << ' '
              << p.fCurrentFromPhotons << ' '
              << p.fPhotonCountFromPhotons << ' '
              << p.fPhotonsFromMeasuredCurrent << ' '
              << p.fPhotonsFromMeasuredPhotonCount;
  }

}


#endif
