// $Id: CurrentModel.h 1849 2011-01-31 06:39:54Z darko $
#ifndef _lidar_CurrentModel_h_
#define _lidar_CurrentModel_h_
#include <lidar/Point.h>
#include <lidar/Trace.h>
#include <iostream>
#include <cmath>


namespace lidar {

  class CurrentModel {
  public:
    CurrentModel() : fConversion(0), fBaseline(0), fVariance(0) { }

    CurrentModel(const double conv, const double base, const double var)
      : fConversion(conv), fBaseline(base), fVariance(var) { }

    CurrentModel GetRescaledModel(const unsigned int nShots) const;

    double EstimateCurrent(const double photon) const
    { return fConversion * photon + fBaseline; }

    /// Inverse of the EstimateCurrent
    double EstimatePhoton(const double current) const
    { return (current - fBaseline) / fConversion; }

    double
    GetDeviance(const Point& d, const double photon)
      const
    {
      const double diff = EstimateCurrent(photon) - d.fCurrent;
      const double v = fVariance + photon;
      return diff*diff/v + std::log(2*M_PI*v);
    }

    std::pair<double, double> GetD12(const Point& d, const double photon) const;

    std::pair<double, double> GetD012(const Point& d, const double q) const;

    bool IsPhysical() const
    { return fConversion > 0 && fBaseline > 0 && fVariance > 0; }

    std::ostream& Dump(std::ostream& os, const std::string& ext = "") const;

    double fConversion;
    double fBaseline;
    double fVariance;
  };


  std::ostream& operator<<(std::ostream& os, const CurrentModel& p);

}


#endif
