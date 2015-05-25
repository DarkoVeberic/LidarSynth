// $Id: PhotonCountModel.cc 1849 2011-01-31 06:39:54Z darko $
#include <lidar/PhotonCountModel.h>

using namespace std;


namespace lidar {

  PhotonCountModel
  PhotonCountModel::GetRescaledModel(const unsigned int nShots)
    const
  {
    PhotonCountModel m(*this);
    m.fDeadFraction *= nShots;
    return m;
  }


  pair<double, double>
  PhotonCountModel::GetD12(const Point& d, const double photon)
    const
  {
    const double c = EstimateCount(photon);
    const double w = 1 + fDeadFraction * photon;
    const double cm = c - d.fPhotonCount;
    const double t = 2*c*cm;
    return make_pair(
      t,
      2*(d.fPhotonCount/(w*w) - t*fDeadFraction/w)
    );
  }


  pair<double, double>
  PhotonCountModel::GetD012(const Point& /*d*/, const double q)
    const
  {
    const double q2d = q*q*fDeadFraction;
    const double w = 1 + q2d;
    const double w2 = w*w;
    return make_pair(
      4*q / w2,
      (4 - 12*q2d) / (w2*w)
    );
  }


  ostream&
  PhotonCountModel::Dump(ostream& os, const string& ext)
    const
  {
    return os << "deadFraction" << ext << '=' << fDeadFraction;
  }


  ostream&
  operator<<(ostream& os, const PhotonCountModel& m)
  {
    return os << m.fDeadFraction;
  }

}
