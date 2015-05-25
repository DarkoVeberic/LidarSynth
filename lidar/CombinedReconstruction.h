// $Id: CombinedReconstruction.h 1849 2011-01-31 06:39:54Z darko $
#ifndef _lidar_CombinedReconstruction_h_
#define _lidar_CombinedReconstruction_h_
#include <lidar/Trace.h>
#include <lidar/PhotonTrace.h>
#include <lidar/MeasurementModel.h>
//#include <lidar/GlobalMinimizationResult.h>
#include <lidar/CombinedReconstructionTrace.h>


namespace lidar {

  CombinedReconstructionTrace
  CombinedReconstruction(const Trace& trace, const PhotonTrace& pTrace,
                         const MeasurementModel& m);


  /*inline
  CombinedReconstructionTrace
  CombinedReconstruction(const Trace& trace, const PhotonTrace& pTrace,
                         const GlobalMinimizationResult& r)
  {
    return CombinedReconstruction(trace, pTrace, MeasurementParameters(r));
  }*/

}


#endif
