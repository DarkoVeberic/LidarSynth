// $Id: TemporaryModify.h 1798 2010-12-20 10:20:43Z darko $
#ifndef _utl_TemporaryModify_h_
#define _utl_TemporaryModify_h_

namespace utl {

  template<typename T>
  class TemporaryModify {
  public:
    TemporaryModify(T& what, const T& newValue)
      : fWhat(what), fOldValue(what) { fWhat = newValue; }
    ~TemporaryModify() { fWhat = fOldValue; }
  private:
    T& fWhat;
    const T fOldValue;
  };

}

#endif
