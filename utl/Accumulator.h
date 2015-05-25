/**
  \file Accumulator.h

  \author Darko Veberic
  \date 24 Aug 2008

  \version $Id: Accumulator.h 3567 2015-03-16 06:13:12Z veberic $
*/

#ifndef _utl_Accumulator_h_
#define _utl_Accumulator_h_

#include <vector>
#include <deque>
#include <cmath>
#include <functional>
#include <algorithm>
#include <utl/Math.h>
#include <utl/SafeBoolCast.h>


namespace utl {
namespace Accumulator {

  class Count {
  public:
    Count(const unsigned int init = 0) : fCount(init) { }
    void operator()() { ++fCount; }
    unsigned int operator++() { operator()(); return fCount; }
    unsigned int GetCount() const { return fCount; }
    void Clear(const unsigned int init = 0) { fCount = init; }
  private:
    unsigned int fCount;
  };


  template<typename T, class Container = std::vector<T> >
  class Collect : public std::unary_function<T, void> {
  public:
    void operator()(const T& x) { fCollection.push_back(x); }

    Container& GetCollection() { return fCollection; }

    const Container& GetCollection() const { return fCollection; }

    template<class Order>
    void Sort(Order ord)
    { std::sort(fCollection.begin(), fCollection.end(), ord); }

    void Clear() { fCollection.clear(); }

  private:
    Container fCollection;
  };


  template<typename T>
  class Min : public std::unary_function<T, void> {
  public:
    typedef T Type;
    Min(const T first) : fMin(first) { }
    void operator()(const T x) { if (fMin > x) fMin = x; }
    T GetMin() const { return fMin; }
    void Clear(const T x) { fMin = x; }
  protected:
    Min() : fMin(T()) { }
    void Clear() { fMin = T(); }
  private:
    T fMin;
  };


  template<typename T>
  class Max : public std::unary_function<T, void> {
  public:
    typedef T Type;
    Max(const T first) : fMax(first) { }
    void operator()(const T x) { if (fMax < x) fMax = x; }
    T GetMax() const { return fMax; }
    void Clear(const T x) { fMax = x; }
  protected:
    Max() : fMax(T()) { }
    void Clear() { fMax = T(); }
  private:
    T fMax;
  };


  template<typename T>
  class MinMax : public Min<T>, public Max<T> {
  public:
    typedef T Type;
    MinMax(const T first) : Min<T>(first), Max<T>(first) { }
    MinMax(const T firstMin, const T firstMax) : Min<T>(firstMin), Max<T>(firstMax) { }
    void operator()(const T x) { Min<T>::operator()(x); Max<T>::operator()(x); }
    void Clear(const T x) { Clear(x, x); }
    void Clear(const T min, const T max) { Min<T>::Clear(min); Max<T>::Clear(max); }
  protected:
    MinMax() : Min<T>(), Max<T>() { }
    void Clear() { Min<T>::Clear(); Max<T>::Clear(); }
  };


  class Sum : public std::unary_function<double, void> {
  public:
    Sum(const double init = 0) : fSumX(init) { }
    void operator()(const double x) { fSumX += x; }
    double GetMean(const int n) const { return fSumX / n; }
    double GetSum() const { return fSumX; }
    void Clear(const double init = 0) { fSumX = init; }
  private:
    double fSumX;
  };


  class Mean : public Count, public Sum {
  public:
    void operator()(const double x) { Count::operator()(); Sum::operator()(x); }
    double GetMean() const { return Sum::GetMean(Count::GetCount()); }
    void Clear() { Count::Clear(); Sum::Clear(); }
  };


  template<typename T>
  class MinMaxMean : public Min<T>, public Max<T>, public Mean {
  public:
    typedef T Type;
    MinMaxMean(const T first) : Min<T>(first), Max<T>(first) { Mean::operator()(first); }
    void operator()(const T x) { Min<T>::operator()(x); Max<T>::operator()(x); Mean::operator()(x); }
    void Clear(const T x) { Min<T>::Clear(x); Max<T>::Clear(x); Mean::Clear(); Mean::operator()(x); }
  protected:
    MinMaxMean() { }
    void Clear() { Min<T>::Clear(); Max<T>::Clear(); Mean::Clear(); }
  };


  class VarN : public Sum {
  public:
    void operator()(const double x) { Sum::operator()(x); fX2(Sqr(x)); }
    double GetVar(const int n) const
    { return (fX2.GetSum() - Sqr(GetSum())/n) / (n - 1); }
    double GetSumOfSquares() const { return fX2.GetSum(); }
    void Clear() { Sum::Clear(); fX2.Clear(); }
  protected:
    Sum fX2;
  };


  class StdN : public VarN {
  public:
    void operator()(const double x) { VarN::operator()(x); }
    double GetStd(const int n) const { return std::sqrt(GetVar(n)); }
  };


  class Var : public Mean {
  public:
    void operator()(const double x) { Mean::operator()(x); fVar(x); }
    double GetVar() const
    { return fVar.GetVar(GetCount()); }
    double GetSumOfSquares() const { return fVar.GetSumOfSquares(); }
    void Clear() { Mean::Clear(); fVar.Clear(); }
  protected:
    VarN fVar;
  };


  /**
    \class Std Accumulator.h "utl/Accumulator.h"

    \brief Accumulates and calculates standard deviation

    Typical usage:
    \code
    vector<double> v;
    utl::Accumulator::Std sigma;
    for_each(v.begin(), v.end(), sigma);
    cout << sigma.GetStd() << endl;
    \endcode
    or in within some loop
    \code
    vector<double> v;
    utl::Accumulator::Std sigma;
    for (vector<double>::const_iterator it = v.begin(); it != v.end(); ++it) {
      ...
      sigma(*it);
      ...
    } 
    \endcode
  */
  class Std : public Var {
  public:
    void operator()(const double x) { Var::operator()(x); }
    double GetStd() const { return std::sqrt(GetVar()); }
  };


  /*template<typename T>
  class Covariance : public Count {
  public:
    explicit Covariance(const unsigned int nVar)
      : Count(), fSumX(nVar), fSumXY(Sqr(nVar)) { }

    void
    operator()(const std::vector<T>& values)
    {
      Count::operator()();
      for (unsigned int i = 0, n = fSumX.size(); i < n; ++i) {
        fSumX[i] += values[i];
        for (unsigned int j = 0; j <= i; ++j)
          fSumXY[n*i + j] += values[i] * values[j];
      }
    }

    std::vector<double>
    GetMeans()
      const
    {
      std::vector<double> m(fSumX.size());
      for (unsigned int i = 0; i < fSumX.size(); ++i)
        m[i] = double(fSumX[i]) / GetCount();
      return m;
    }

    std::vector<double>
    GetVars()
      const
    {
      const unsigned int n = fSumX.size();
      std::vector<double> v(n);
      for (unsigned int i = 0; i < n; ++i)
        v[i] = (fSumXY[(n+1)*i] - double(Sqr(fSumX[i]))/GetCount()) / (GetCount() - 1);
      return v;
    }

    std::vector<double>
    GetStds()
      const
    {
      std::vector<double> s = GetVars();
      for (double& x : s)
        x = std::sqrt(x);
      return s;
    }

    double
    GetCovariance(const unsigned int i, const unsigned int j)
      const
    {
      std::pair<const unsigned int&, const unsigned int&> ji = std::minmax(i, j);
      const unsigned int& ii = ji.second;
      const unsigned int& jj = ji.first;
      const unsigned int n = fSumX.size();
      return (fSumXY[n*ii + jj] - double(fSumX[ii]*fSumX[jj])/GetCount()) / (GetCount() - 1);
    }

    std::vector<std::vector<double> >
    GetCovariance()
      const
    {
      const unsigned int n = fSumX.size();
      std::vector<std::vector<double>> c(n, std::vector<double>(n));
      for (unsigned int i = 0; i < n; ++i)
        for (unsigned int j = 0; j < n; ++j)
          c[i][j] = GetCovariance(i, j);
      return c;
    }

  private:
    std::vector<T> fSumX;
    std::vector<T> fSumXY;
  };*/


  //


  class LinearFit : public Count {
  public:
    void operator()(const double x, const double y)
    { Count::operator()(); fSx(x); fSy(y); fSxx(x*x); fSxy(x*y); }

    double GetDeterminant() const { return GetCount()*fSxx.GetSum() - Sqr(fSx.GetSum()); }

    std::pair<double, double>
    GetCoefficients()
      const
    {
      const double invD = 1 / GetDeterminant();
      return std::make_pair(
        (fSxx.GetSum()*fSy.GetSum() - fSx.GetSum()*fSxy.GetSum()) * invD,
        (GetCount()*fSxy.GetSum() - fSx.GetSum()*fSy.GetSum()) * invD
      );
    }

    void Clear() { Count::Clear(); fSx.Clear(); fSy.Clear(); fSxx.Clear(); fSxy.Clear(); }

  protected:
    Sum fSx;
    Sum fSy;
    Sum fSxx;
    Sum fSxy;
  };


  class LinearFitWithVar : public LinearFit {
  public:
    void operator()(const double x, const double y) { LinearFit::operator()(x, y); fSyy(y*y); }

    double
    GetVar()
      const
    {
      return (fSyy.GetSum() -
              (GetCount()*Sqr(fSxy.GetSum()) -
               2*fSx.GetSum()*fSxy.GetSum()*fSy.GetSum() +
               fSxx.GetSum()*Sqr(fSy.GetSum())) / GetDeterminant()) / (GetCount() - 2);
    }

    void Clear() { LinearFit::Clear(); fSyy.Clear(); }

  private:
    Sum fSyy;
  };


  //


  class Validator {
  public:
    Validator() : fIsValid(false) { }
    bool IsValid() const { return fIsValid; }
    void Validate(const bool state = true) { fIsValid = state; }
    void Clear() { fIsValid = false; }
  private:
    bool fIsValid;
  };

  /**
    This class enables the usage of the uninitialized versions
    (default empty constructors) of the accumulators that require
    the first value to be passed in the constructor.

    Example:
    \code
    Accumulator::Safe<Accumulator::Min<int> > min;

    if (min)
      cout << "min is initialized" << endl;
    else
      cout << "min is uninitialized" << endl;

    min(13);

    if (min)
      cout << "min is initialized" << endl;
    else
      cout << "min is uninitialized" << endl;

    cout << min.GetMin() << endl;
    \endcode

    This example prints
    \code
    min is uninitialized
    min is initialized
    13
    \endcode
  */
  template<typename AccumulatorT>
  class Safe : public SafeBoolCast<Safe<AccumulatorT> >,
               public AccumulatorT, private Validator {
  public:
    void
    operator()(const typename AccumulatorT::Type x)
    {
      if (*this)
        AccumulatorT::operator()(x);
      else {
        AccumulatorT::Clear(x);
        Validate();
      }
    }

    bool BoolCast() const { return IsValid(); }

    void Clear() { AccumulatorT::Clear(); Validator::Clear(); }
  };

} // namespace Accumulator
} // namespace utl


#endif
