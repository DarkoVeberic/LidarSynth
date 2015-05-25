#ifndef _utl_Identity_h_
#define _utl_Identity_h_

namespace utl {

template<typename T>
struct Identity {
  typedef T Type;
  const T& operator()(const T& t) const
  { return t; }
};

}

#endif
