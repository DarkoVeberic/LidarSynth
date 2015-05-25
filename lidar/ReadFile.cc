// $Id: ReadFile.cc 1827 2011-01-14 15:19:05Z darko $
#include <lidar/Trace.h>
#include <utl/io/ZFStream.h>
#include <utl/io/Read.h>

using namespace std;


namespace lidar {

  void
  ReadFile(const string& filename, Trace& trace)
  {
    using namespace utl::io;
    ZIFStream izfs(filename.c_str());
    Read read(izfs);
    try {
      read.GetLines(trace.fTrace);
    } catch (...) {
      cerr << "read " << trace.fTrace.size() << " lines" << endl;
      throw;
    }
    Trace::Type& t = trace.fTrace;
    for (unsigned int i = 0; i < t.size(); ++i)
      t[i].fIndex = i;
  }

}
