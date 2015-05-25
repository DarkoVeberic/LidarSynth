#ifndef _utl_Random_h_
#define _utl_Random_h_

//#include <cmath>
#include <TRandom3.h>


namespace utl {

namespace Random {

  inline
  double
  Exponential(const double mean)
  {
    //return -std::log(1 - gRandom->Rndm()) * mean;
    return gRandom->Exp(mean);
  }


  double
  Normal()
  {
    return gRandom->Gaus(0, 1);
    /*static bool generate = true;
    static double other;
    if (generate) {
      const double x = std::sqrt(-2*std::log(1 - rand()));
      const double y = 2*M_PI*rand();
      other = x * std::sin(y);
      generate = false;
      return x * std::cos(y);
    }
    generate = true;
    return other;*/
  }


  inline
  double
  Normal(const double mean, const double sigma = 1)
  {
    return Normal() * sigma + mean;
  }


  unsigned int
  Poisson(const double mean)
  {
    return gRandom->Poisson(mean);
    /*if (mean > 100)
      return round(Normal(rand, mean, sqrt(mean)));

    const double target = rand() * std::exp(mean);
    int i = 0;
    double sum = 1;
    double term = 1;
    while (sum < target) {
      ++i;
      term *= mean/i;
      sum += term;
    }
    return i;*/
  }

}

}


#endif
