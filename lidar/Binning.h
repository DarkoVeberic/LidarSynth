// $Id: Binning.h 1823 2011-01-12 15:43:48Z darko $
#ifndef _lidar_Binning_h_
#define _lidar_Binning_h_

#include <lidar/Trace.h>
#include <vector>
#include <numeric>
#include <algorithm>


namespace lidar {

  class Binning {
  public:
    virtual unsigned int GetIndex(const Point& p) const = 0;

    double GetWeight(const Point& p) const
    { return GetWeight(this->GetIndex(p)); }

    unsigned int GetNBins() const { return fWeight.size(); }

    unsigned int GetNEmptyBins() const
    { return std::count(fWeight.begin(), fWeight.end(), 0.); }

    unsigned int GetNNonemptyBins() const { return GetNBins() - GetNEmptyBins(); }

    double GetNorm(const Trace& trace) const;

    double GetNorm() const
    { return std::accumulate(fWeight.begin(), fWeight.end(), 0.); }

    double GetMinWeight() const
    { return *std::min_element(fWeight.begin(), fWeight.end()); }

    double GetMinNonzeroWeight() const;

    double GetMaxWeight() const
    { return *std::max_element(fWeight.begin(), fWeight.end()); }

    double GetMeanWeight() const
    { return GetNorm() / GetNBins(); }

    double GetMeanNonzeroWeight() const
    { return GetNorm() / GetNNonemptyBins(); }

    void Dump() const;

  protected:
    void Bin(const Trace& trace);

  private:
    double GetWeight(const unsigned int i) const { return fWeight[i]; }

    void ConvertCountsToWeights(const std::vector<unsigned int>& count, const unsigned int n);

    unsigned int fNData;
    std::vector<double> fWeight;
  };

}


#endif
