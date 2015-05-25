// $Id: GlobalMinimizationResult.h 1850 2011-01-31 16:51:45Z darko $
#ifndef _lidar_GlobalMinimizationResult_h_
#define _lidar_GlobalMinimizationResult_h_
#include <lidar/MeasurementModel.h>
#include <iostream>


namespace lidar {

  class MeasurementModel;


  struct GlobalMinimizationResult {
    GlobalMinimizationResult(const bool converged = false)
      : fIsConverged(converged), fMeasurementModel(), fMeasurementModelError() { }

    GlobalMinimizationResult(const bool converged,
                             const MeasurementModel& m,
                             const MeasurementModel& mErr = MeasurementModel()) :
      fIsConverged(converged),
      fMeasurementModel(m),
      fMeasurementModelError(mErr)
    { }

    std::ostream& Dump(std::ostream& os) const;

    bool fIsConverged;
    MeasurementModel fMeasurementModel;
    MeasurementModel fMeasurementModelError;
  };


  std::ostream& operator<<(std::ostream& os, const GlobalMinimizationResult& r);

}


#endif
