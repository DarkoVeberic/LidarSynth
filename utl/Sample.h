#ifndef _utl_Sample_h_
#define _utl_Sample_h_

#include <utl/Stat.h>
#include <numeric>
#include <iterator>
#include <vector>


namespace utl {

namespace Sample {

  template<typename Iterator>
  inline
  double
  Mean(const Iterator begin, const Iterator end, const double init = 0)
  {
    return std::accumulate(begin, end, init) / std::distance(begin, end);
  }


  template<typename Iterator>
  std::pair<double, double>
  MeanVariance(const Iterator begin, const Iterator end)
  {
    const unsigned int n = std::distance(begin, end);
    double sum = 0;
    double sum2 = 0;
    for ( ; begin != end; ++begin) {
      const double d = *begin;
      sum += d;
      sum2 += d * d;
    }
    return std::make_pair(sum / n, Stat::SampleVariance(n, sum, sum2));
  }


  template<typename Iterator>
  double
  Variance(Iterator begin, const Iterator end, const double mean = 0)
  {
    const unsigned int n = std::distance(begin, end);
    double sum = 0;
    double sum2 = 0;
    for ( ; begin != end; ++begin) {
      const double d = *begin - mean;
      sum += d;
      sum2 += d * d;
    }
    return Stat::SampleVariance(n, sum, sum2);
  }


  template<typename Iterator>
  inline
  double
  AccurateVariance(const Iterator begin, const Iterator end)
  {
    return Variance(begin, end, Mean(begin, end));
  }


  template<typename Iterator>
  double
  Skewness(Iterator begin, const Iterator end, const double mean)
  {
    const unsigned int n = std::distance(begin, end);
    double sum = 0;
    double sum2 = 0;
    double sum3 = 0;
    for ( ; begin != end; ++begin) {
      const double diff = *begin - mean;
      sum += diff;
      const double diff2 = diff * diff;
      sum2 += diff2;
      sum3 += diff2 * diff;
    }
    return Stat::Skewness(n, Stat::SampleVariance(n, sum, sum2), sum3 / n);
  }


  template<typename Iterator>
  inline
  double
  Skewness(const Iterator begin, const Iterator end)
  {
    return Skewness(begin, end, Mean(begin, end));
  }


  template<typename Iterator>
  double
  ExcessKurtosis(Iterator begin, const Iterator end, const double mean)
  {
    const unsigned int n = std::distance(begin, end);
    double sum = 0;
    double sum2 = 0;
    double sum4 = 0;
    for ( ; begin != end; ++begin) {
      const double diff = *begin - mean;
      sum += diff;
      const double diff2 = diff * diff;
      sum2 += diff2;
      sum4 += diff2 * diff2;
    }
    return Stat::ExcessKurtosis(n, Stat::SampleVariance(n, sum, sum2), sum4 / n);
  }


  template<typename Iterator>
  inline
  double
  ExcessKurtosis(const Iterator begin, const Iterator end)
  {
    return ExcessKurtosis(begin, end, Mean(begin, end));
  }


  namespace {

    inline
    std::pair<double, double>
    LinearFitCoefficients(const double s1, const double sx, const double sy,
                          const double sxx, const double sxy)
    {
      const double invD = 1 / (s1*sxx - sx*sx);
      const double a = (sxx*sy - sx*sxy) * invD;
      const double b = (s1*sxy - sx*sy) * invD;
      return std::make_pair(a, b);
    }

  }


  template<typename Iterator>
  std::pair<double, double>
  LinearFit(Iterator begin, const Iterator end)
  {
    const unsigned int n = std::distance(begin, end);
    unsigned int x = 0;
    double sy = 0;
    double sxy = 0;
    for ( ; begin != end; ++begin) {
      const double y = *begin;
      sy += y;
      sxy += x*y;
      ++x;
    }
    const double sx = 0.5 * n*(n - 1);
    const double sxx = (1./6) * n*(n - 1)*(2*n - 1);
    return LinearFitCoefficients(n, sx, sy, sxx, sxy);
  }


  template<typename Iterator>
  std::pair<double, double>
  LinearFit(Iterator xBegin, const Iterator xEnd, Iterator yBegin)
  {
    const unsigned int n = std::distance(xBegin, xEnd);
    double sx = 0;
    double sy = 0;
    double sxx = 0;
    double sxy = 0;
    for ( ; xBegin != xEnd; ++xBegin, ++yBegin) {
      const double x = *xBegin;
      sx += x;
      sxx += x*x;
      const double y = *yBegin;
      sy += y;
      sxy += x*y;
    }
    return LinearFitCoefficients(n, sx, sy, sxx, sxy);
  }


  struct LinearFunctionWithVariance {
    double operator()(const double x) const
    { return f0 + x * f1; }

    double Inverse(const double y) const
    { return (y - f0) / f1; }

    double f0;
    double f1;
    double fVariance;
  };


  template<typename Iterator>
  LinearFunctionWithVariance
  LinearFitWithVariance(Iterator xBegin, const Iterator xEnd, Iterator yBegin)
  {
    const unsigned int n = std::distance(xBegin, xEnd);
    double sx = 0;
    double sy = 0;
    double sxx = 0;
    double sxy = 0;
    double syy = 0;
    for ( ; xBegin != xEnd; ++xBegin, ++yBegin) {
      const double x = *xBegin;
      sx += x;
      sxx += x*x;
      const double y = *yBegin;
      sy += y;
      sxy += x*y;
      syy += y*y;
    }
    const double invD = 1 / (n*sxx - sx*sx);
    LinearFunctionWithVariance fit;
    fit.f0 = (sxx*sy - sx*sxy) * invD;
    fit.f1 = (n*sxy - sx*sy) * invD;
    fit.fVariance = (syy - (n*sxy*sxy - 2*sx*sxy*sy + sxx*sy*sy) * invD) / (n - 2);
    return fit;
  }


  template<typename Iterator>
  std::pair<double, double>
  LinearFit(Iterator xBegin, const Iterator xEnd, Iterator yBegin, Iterator yErrBegin)
  {
    //const unsigned int n = std::distance(xBegin, xEnd);
    double s1 = 0;
    double sx = 0;
    double sy = 0;
    double sxx = 0;
    double sxy = 0;
    for ( ; xBegin != xEnd; ++xBegin, ++yBegin, ++yErrBegin) {
      const double sigma = *yErrBegin;
      const double invSigma2 = 1 / (sigma*sigma);
      s1 += invSigma2;
      const double x = *xBegin;
      sx += x * invSigma2;
      sxx += x*x * invSigma2;
      const double y = *yBegin;
      sy += y * invSigma2;
      sxy += x*y * invSigma2;
    }
    return LinearFitCoefficients(s1, sx, sy, sxx, sxy);
  }


  template<typename Iterator>
  double
  DetrendedVariance(Iterator begin, const Iterator end)
  {
    const std::pair<double, double> ab = LinearFit(begin, end);
    const unsigned int n = std::distance(begin, end);
    unsigned int x = 0;
    double sum = 0;
    double sum2 = 0;
    for ( ; begin != end; ++begin) {
      const double y = ab.first + ab.second*x;
      const double d = *begin - y;
      sum += d;
      sum2 += d * d;
      ++x;
    }
    return Stat::SampleVariance(n, sum, sum2) * (n - 1) / (n - 2);
  }


  template<typename Iterator>
  inline
  double
  ScalarProduct(Iterator xBegin, const Iterator xEnd, Iterator yBegin, const double mean = 0)
  {
    double sum = 0;
    for ( ; xBegin != xEnd; xBegin++, yBegin++)
      sum += (*xBegin - mean) * (*yBegin - mean);
    return sum;
  }


  template<typename Iterator>
  std::vector<double>
  Autocorrelation(const Iterator begin, const Iterator end, const std::size_t length,
                  const double mean, const double variance)
  {
    std::vector<double> ac;
    const std::size_t n = std::distance(begin, end);
    Iterator it = begin;
    for (std::size_t i = 0; i < length; ++i, ++it)
      ac.push_back(ScalarProduct(it, end, begin, mean) / ((n - i) * variance));
    return ac;
  }


  template<typename Iterator>
  inline
  std::vector<double>
  Autocorrelation(const Iterator begin, const Iterator end, const std::size_t length)
  {
    const double mean = Mean(begin, end);
    const double var = Variance(begin, end, mean);
    return Autocorrelation(begin, end, length, mean, var);
  }

}

}


#endif
