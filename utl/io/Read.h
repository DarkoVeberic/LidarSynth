#ifndef _utl_io_Read_h_
#define _utl_io_Read_h_

#include <utl/SafeBoolCast.h>
#include <vector>
#include <iostream>
#include <string>
#include <boost/lexical_cast.hpp>


namespace utl {

namespace io {

class Read : public SafeBoolCast<Read> {
public:
  Read() : fStream(0) { }

  Read(std::istream& is) : fStream(&is) { }

  void SetStream(std::istream& is) { fStream = &is; }

  std::istream& GetStream() const { return *fStream; }

  void Clear() { fStream = 0; }

  template<typename T>
  bool Get(T& t)
  { return fStream && (GetStream() >> t); }

  template<typename T>
  bool
  GetLine(T& t)
  {
    std::string line;
    if (!fStream || !std::getline(*fStream, line))
      return false;
    t = boost::lexical_cast<T>(line);
    return true;
  }

  template<typename T>
  void
  GetLines(std::vector<T>& v)
  {
    if (!fStream)
      return;
    std::string line;
    while (std::getline(*fStream, line))
      if (!line.empty())
        v.push_back(boost::lexical_cast<T>(line));
  }

  bool BoolCast() const { return fStream && *fStream; }

private:
  std::istream* fStream;
};

}

}

#endif
