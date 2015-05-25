// $Id: CurrentModel.cc 1850 2011-01-31 16:51:45Z darko $
#include <lidar/CurrentModel.h>

using namespace std;


namespace lidar {

  CurrentModel
  CurrentModel::GetRescaledModel(const unsigned int nShots)
    const
  {
    CurrentModel m(*this);
    m.fBaseline /= nShots;
    m.fVariance /= nShots;
    return m;
  }


  pair<double, double>
  CurrentModel::GetD12(const Point& d, const double photon)
    const
  {
    const double p2 = photon*photon;
    const double diff = d.fCurrent - EstimateCurrent(photon);
    const double v = fVariance + photon;
    const double v2 = v*v;
    const double x = d.fCurrent - fBaseline + fConversion*fVariance;
    return make_pair(
      p2*(v - diff*(diff + 2*v*fConversion)) / v2,
      p2*(2*x*x - v) / (v2*v)
    );
  }


  pair<double, double>
  CurrentModel::GetD012(const Point& d, const double q)
    const
  {
    const double q2 = q*q;
    const double q2a = q2 * fConversion;
    const double q4a2 = q2a*q2a;
    const double v = q2 + fVariance;
    const double v2 = v*v;
    const double ab = d.fCurrent - fBaseline;
    const double ab2 = ab*ab;
    const double ae = EstimateCurrent(q2);
    const double a2 = d.fCurrent*d.fCurrent;
    const double qq = 3*q2a + fBaseline;
    return make_pair(
      2*q * (v + q4a2 - ab2 + 2*(ae - d.fCurrent)*fConversion*fVariance) / v2,
      2*(
          q2*(q4a2 - q2 + 3*ab2) -
          fVariance*(
                      a2 - 2*d.fCurrent*qq - 3*q4a2 + fBaseline*(6*q2a + fBaseline) -
                      (1 + 2*(qq - d.fCurrent)*fConversion)*fVariance
                    )
        ) / (v2*v)
    );
  }


  ostream&
  CurrentModel::Dump(ostream& os, const string& ext)
    const
  {
    return os << "conversion" << ext << '=' << fConversion << " "
                 "baseline" << ext << '=' << fBaseline << " "
                 "variance" << ext << '=' << fVariance;
  }


  ostream&
  operator<<(ostream& os, const CurrentModel& m)
  {
    return os << m.fConversion << ' '
              << m.fBaseline << ' '
              << m.fVariance;
  }

}
