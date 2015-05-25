#ifndef _utl_SafeBoolCast_h_
#define _utl_SafeBoolCast_h_

namespace utl {

  /**
    \brief Deliver safe bool cast for bool-like interface

    Inspired by the boost implementation of the safe bool cast
    in boost::shared_ptr class. The implementation is adapted from
    http://everything2.com/e2node/C%252B%252B%253A%2520true%2520or%2520false%253F

    A simpler explanation of the problem with automatic conversion to bool and
    of the trick used to avoid the conversion to bool in the places where it
    doesn't make sense can be found in
    http://www.devx.com/cplus/10MinuteSolution/32145/1954

    \version $Id: SafeBoolCast.h 1414 2010-02-22 20:32:33Z darko $
    \author Darko Veberic
    \date 25 Sep 2008
    \ingroup stl
  */
  // template with T = void tries virtual call to BoolCast()
  template<typename T = void>
  class SafeBoolCast {
  private:
    struct Finesse {
      int ComparisonIsNotAllowed;
    };
    typedef const int Finesse::* BoolType;
  public:
    operator BoolType()
      const
    {
      return static_cast<const T*>(this)->BoolCast() ?
        &Finesse::ComparisonIsNotAllowed : 0;
    }
  protected:
    ~SafeBoolCast() { }
  };

  template<>
  class SafeBoolCast<void> {
  private:
    struct Finesse {
      int ComparisonIsNotAllowed;
    };
    typedef const int Finesse::* BoolType;
  public:
    operator BoolType()
      const
    {
      return BoolCast() ?
        &Finesse::ComparisonIsNotAllowed : 0;
    }
  protected:
    // force implementation
    virtual bool BoolCast() const = 0;
    virtual ~SafeBoolCast() { }
  };

  // this is just to get compiler error
  template<typename T, typename U>
  bool operator==(const SafeBoolCast<T>& a, const SafeBoolCast<U>&)
  { a.ComparisonIsNotAllowed(); return false; }

  template<typename T, typename U>
  bool operator!=(const SafeBoolCast<T>& a, const SafeBoolCast<U>&)
  { a.ComparisonIsNotAllowed(); return false; }

}


#endif
