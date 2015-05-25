// $Id: EstimatePhotonCountModel.h 1849 2011-01-31 06:39:54Z darko $
#ifndef _lidar_EstimatePhotonCountModel_h_
#define _lidar_EstimatePhotonCountModel_h_
#include <lidar/PhotonCountModel.h>
#include <lidar/Trace.h>


namespace lidar {

  PhotonCountModel
  EstimatePhotonCountModel(const Trace& trace, const double upperFitFraction = 0.3);

}


#endif
