#ifndef _utl_Stat_h_
#define _utl_Stat_h_

#include <cmath>


namespace utl {

namespace Stat {

  inline
  double
  SampleVariance(const unsigned int n, const double sum, const double sum2)
  {
    return (sum2 - sum*sum/n) / (n - 1);
  }


  inline
  double
  Skewness(const unsigned int n, const double biasedVariance, const double m3)
  {
    const double variance3 = std::pow(biasedVariance, 3);
    return std::sqrt(n * (n - 1.) / variance3) / (n - 2) * m3;
  }


  inline
  double
  ExcessKurtosis(const unsigned int n, const double unbiasedVariance,
                 const double m4)
  {
    const double variance2 = unbiasedVariance * unbiasedVariance;
    return ((n + 1.)*n*n/(n - 1) * m4 / variance2 -
            3*(n - 1.)*(n - 1)) / ((n - 2.) * (n - 3));
  }

}

}


#endif
