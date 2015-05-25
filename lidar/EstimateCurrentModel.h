// $Id: EstimateCurrentModel.h 1849 2011-01-31 06:39:54Z darko $
#ifndef _lidar_EstimateCurrentModel_h_
#define _lidar_EstimateCurrentModel_h_
#include <lidar/CurrentModel.h>
#include <lidar/PhotonCountModel.h>
#include <lidar/Trace.h>


namespace lidar {

  CurrentModel
  EstimateCurrentModel(const Trace& trace, const double lowerFitFraction = 0.11);


  CurrentModel
  EstimateCurrentModel(const Trace& trace, const PhotonCountModel& pcp,
                       const double lowerFitFraction = 0.11);

}


#endif
