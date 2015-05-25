// $Id: EstimateMeasurementModel.h 1849 2011-01-31 06:39:54Z darko $
#ifndef _lidar_EstimateMeasurementModel_h_
#define _lidar_EstimateMeasurementModel_h_
#include <lidar/Trace.h>
#include <lidar/EstimateCurrentModel.h>
#include <lidar/EstimatePhotonCountModel.h>


namespace lidar {

  inline
  MeasurementModel
  EstimateMeasurementModel(const Trace& trace,
                           const double currentLowerFitFraction = 0.11,
                           const double photonCountUpperFitFraction = 0.3)
  {
    const PhotonCountModel pcm =
      EstimatePhotonCountModel(trace, photonCountUpperFitFraction);
    const CurrentModel cm =
      EstimateCurrentModel(trace, pcm, currentLowerFitFraction);
    return MeasurementModel(cm, pcm);
  }

}


#endif
