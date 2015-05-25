// $Id: FilterSaturation.h 1823 2011-01-12 15:43:48Z darko $
#ifndef _lidar_FilterSaturation_h_
#define _lidar_FilterSaturation_h_

#include <lidar/Trace.h>


namespace lidar {

  Trace
  FilterSaturation(const Trace& trace,
                   const unsigned int maxADC = (1 << 12) - 1);

}


#endif
