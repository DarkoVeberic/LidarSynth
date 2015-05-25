// $Id: PhotonCountModel.h 1849 2011-01-31 06:39:54Z darko $
#ifndef _lidar_PhotonCountModel_h_
#define _lidar_PhotonCountModel_h_

#include <lidar/Point.h>
#include <lidar/Trace.h>
#include <utl/Math.h>
#include <iostream>
#include <limits>
#include <cmath>


namespace lidar {

  class PhotonCountModel {
  public:
    PhotonCountModel() : fDeadFraction(0), fCountLimit(0) { }

    PhotonCountModel(const double dFrac)
      : fDeadFraction(dFrac), fCountLimit(1/dFrac) { }

    PhotonCountModel GetRescaledModel(const unsigned int nShots) const;

    double EstimateCount(const double photon) const
    { return photon > 0 ? photon / (1 + photon * fDeadFraction) : 0; }

    // Inverse of the EstimateCount
    double
    EstimatePhoton(const unsigned int count)
      const
    {
      if (count < fCountLimit)
        return count / (1 - count * fDeadFraction);
      return std::numeric_limits<double>::quiet_NaN();
    }

    double
    GetDeviance(const Point& d, const double photon)
      const
    {
      const unsigned int m = d.fPhotonCount;
      const double c = EstimateCount(photon);
      return m || c ?
             2*(c - m*std::log(c) + utl::LogFactorial(d.fPhotonCount)) : 0;
    }

    std::pair<double, double> GetD12(const Point& d, const double photon) const;

    std::pair<double, double> GetD012(const Point& /*d*/, const double q) const;

    double GetCountLimit() const { return fCountLimit; }

    bool IsPhysical() const { return fDeadFraction > 0; }

    std::ostream& Dump(std::ostream& os, const std::string& ext = "") const;

    double fDeadFraction;
    double fCountLimit;
  };


  std::ostream& operator<<(std::ostream& os, const PhotonCountModel& m);

}


#endif
