// $Id: ReadFile.h 1827 2011-01-14 15:19:05Z darko $
#ifndef _lidar_ReadFile_h_
#define _lidar_ReadFile_h_
#include <lidar/Point.h>
#include <string>


namespace lidar {

  void ReadFile(const std::string& filename, Trace& trace);

}


#endif
