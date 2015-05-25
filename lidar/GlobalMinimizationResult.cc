// $Id: GlobalMinimizationResult.cc 1849 2011-01-31 06:39:54Z darko $
#include <lidar/GlobalMinimizationResult.h>

using namespace std;


namespace lidar {

  ostream&
  GlobalMinimizationResult::Dump(ostream& os)
    const
  {
    os << "converged=" << fIsConverged << ' ';
    fMeasurementModel.Dump(os, "");
    os << ' ';
    fMeasurementModel.Dump(os, "Err");
    return os;
  }


  ostream&
  operator<<(ostream& os, const GlobalMinimizationResult& r)
  {
    return os << r.fIsConverged << ' '
              << r.fMeasurementModel << ' '
              << r.fMeasurementModelError;
  }

}
