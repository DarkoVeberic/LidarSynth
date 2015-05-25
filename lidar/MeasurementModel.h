// $Id: MeasurementModel.h 1852 2011-02-01 06:49:56Z darko $
#ifndef _lidar_MeasurementModel_h_
#define _lidar_MeasurementModel_h_
#include <lidar/Point.h>
#include <lidar/CurrentModel.h>
#include <lidar/PhotonCountModel.h>
#include <iostream>
#include <utility>


namespace lidar {

  class GlobalMinimizationResult;


  class MeasurementModel {
  public:
    MeasurementModel() : fCurrentModel(), fPhotonCountModel() { }

    MeasurementModel(const CurrentModel& cm, const PhotonCountModel& pcm)
      : fCurrentModel(cm), fPhotonCountModel(pcm) { }

    MeasurementModel(const GlobalMinimizationResult& r);

    MeasurementModel GetRescaledModel(const unsigned int nShots) const
    { return MeasurementModel(fCurrentModel.GetRescaledModel(nShots),
                              fPhotonCountModel.GetRescaledModel(nShots)); }

    void SetModel(const CurrentModel& cm, const PhotonCountModel& pcm)
    { fCurrentModel = cm; fPhotonCountModel = pcm; }

    double
    GetDeviance(const Point& d, const double photon)
      const
    {
      return fCurrentModel.GetDeviance(d, photon) +
             fPhotonCountModel.GetDeviance(d, photon);
    }

    std::pair<double, double> GetArgMinDeviance(const Point& d) const;

    bool IsMinimum(const Point& d, const double pmin, const double eps = 1e-5) const;

    std::ostream& Dump(std::ostream& os, const std::string& ext = "") const
    { fCurrentModel.Dump(os, ext); os << ' '; fPhotonCountModel.Dump(os, ext); return os; }

    bool IsPhysical() const
    { return fCurrentModel.IsPhysical() && fPhotonCountModel.IsPhysical(); }

  //private:
    std::pair<double, double> GetApproximateArgMinDeviance(const Point& d) const;

    double
    GetDRatio(const Point& d, const double photon)
      const
    {
      return /*d.fPhotonCount ?*/
        RatioOfSums(fCurrentModel.GetD12(d, photon),
                    fPhotonCountModel.GetD12(d, photon)) /*:
        RatioOfSums(fCurrentModel.GetD012(d, photon),
                    fPhotonCountModel.GetD012(d, photon))*/;
    }

  private:
    static
    double RatioOfSums(const std::pair<double, double>& p1,
                       const std::pair<double, double>& p2)
    { return (p1.first + p2.first) / (p1.second + p2.second); }

    double IterateNewtonRaphson(unsigned int& steps, const double eps,
                                const Point& d, double p) const;

    //void Complain(const unsigned int steps, const Point& d, const double p) const;

  public:
    CurrentModel fCurrentModel;
    PhotonCountModel fPhotonCountModel;
  };


  inline
  std::ostream& operator<<(std::ostream& os, const MeasurementModel& m)
  { return os << m.fCurrentModel << ' ' << m.fPhotonCountModel; }

}


#endif
