// $Id: MeasurementModel.cc 1852 2011-02-01 06:49:56Z darko $
#include <lidar/MeasurementModel.h>
#include <lidar/GlobalMinimizationResult.h>

using namespace std;


namespace lidar {

  MeasurementModel::MeasurementModel(const GlobalMinimizationResult& r)
  {
    *this = r.fMeasurementModel;
  }


  pair<double, double>
  MeasurementModel::GetApproximateArgMinDeviance(const Point& d)
    const
  {
    const double p0 = fCurrentModel.EstimatePhoton(d.fCurrent);
    const double ab = d.fCurrent - fCurrentModel.fBaseline;
    const double v = fCurrentModel.fVariance;
    const double alpha = fCurrentModel.fConversion;
    const double delta = fPhotonCountModel.fDeadFraction;
    const double k =
      (v - ab*(ab +  2*alpha*v)) / (v*v);
    const double tm = 2*d.fPhotonCount;
    const double p1 = tm / (2 + k + tm*delta);
    const double apd1 = alpha*(1 + p0*delta);
    const double p2 = p0 - (p0 + v) / (apd1*apd1);
    pair<double, double> approx(p1, p2);
    if (p1 < 0)
      approx.first = -1;
    if (p2 < 0)
      approx.second = -1;
    return approx;
  }


  pair<double, double>
  MeasurementModel::GetArgMinDeviance(const Point& pt)
    const
  {
    const pair<double, double> p0 = GetApproximateArgMinDeviance(pt);
    const double eps = 1e-5;
    const unsigned int maxSteps = 20;
    pair<double, double> pdmin(-1, 0);
    if (p0.first > 0) {
      unsigned int steps = maxSteps;
      const double p = IterateNewtonRaphson(steps, eps, pt, p0.first);
      if (steps && p > 0)
        pdmin = make_pair(p, GetDeviance(pt, p));
    }
    if (p0.second > 0) {
      unsigned int steps = maxSteps;
      const double p = IterateNewtonRaphson(steps, eps, pt, p0.second);
      if (steps && p > 0) {
        const double d = GetDeviance(pt, p);
        if (pdmin.first < 0 || pdmin.second > d)
          pdmin = make_pair(p, d);
      }
    }
    if (!pt.fPhotonCount) {
      const double d = GetDeviance(pt, 0);
      if (pdmin.first < 0 || pdmin.second > d)
        pdmin = make_pair(0., d);
    }
    if (pdmin.first == -1) {
      cerr << "no minimum found "
           << pt << ' ' << p0.first << ' ' << p0.second << ' '
           << *this << endl;
      throw;
    }
    return pdmin;
  }


  bool
  MeasurementModel::IsMinimum(const Point& d, const double pmin, const double eps)
    const
  {
    if (pmin < 0)
      return false;
    const double d0 = GetDeviance(d, pmin);
    const double d1 = GetDeviance(d, pmin*(1 - eps));
    const double d2 = GetDeviance(d, pmin*(1 + eps));
    return d0 <= d1 && d0 <= d2;
  }


  double
  MeasurementModel::IterateNewtonRaphson(unsigned int& steps, const double eps,
                                         const Point& d, double p)
    const
  {
    unsigned int s = steps;
    double pOld;
    do {
      pOld = p;
      p -= GetDRatio(d, p);
      --s;
    } while (p >= 0 && fabs(p - pOld) > eps && s);
    steps = s;
    return p;
  }


  /*void
  MeasurementModel::Complain(const unsigned int steps, const Point& d, const double p)
    const
  {
    if (!steps) {
      cerr << "Convergence in Newton-Raphson iteration not reached for "
           << fCurrentModel << ' '
           << fPhotonCountModel << ' ' << d
           << ". Last value: " << p << endl;
    }
  }*/

}
