// $Id: Binning.cc 1824 2011-01-12 18:41:37Z darko $
#include <lidar/Binning.h>

using namespace std;


namespace lidar {

  double
  Binning::GetMinNonzeroWeight()
    const
  {
    vector<double>::const_iterator it = fWeight.begin();
    vector<double>::const_iterator end = fWeight.end();
    double min = 0;
    for ( ; it != end; ++it)
      if (*it > 0) {
        min = *it;
        break;
      }
    if (!min)
      return -1;
    for ( ; it != end; ++it)
      if (*it && min > *it)
        min = *it;
    return min;
  }


  double
  Binning::GetNorm(const Trace& trace)
    const
  {
    const Trace::Type& t = trace.fTrace;
    double sum = 0;
    for (Trace::Type::const_iterator it = t.begin(), end = t.end();
         it != end; ++it)
      sum += this->GetWeight(*it);
    return sum;
  }


  void
  Binning::Dump()
    const
  {
    const unsigned int nne = GetNNonemptyBins();
    for (unsigned int i = 0, n = fWeight.size(); i < n; ++i) {
      const double w = fWeight[i];
      cout << i << ' ' << w << ' ' << (w ? fNData/w/nne : 0) << endl;
    }
  }


  void
  Binning::Bin(const Trace& trace)
  {
    const Trace::Type& t = trace.fTrace;
    fNData = t.size();
    vector<unsigned int> count;
    for (Trace::Type::const_iterator it = t.begin(), end = t.end();
         it != end; ++it) {
      const unsigned int i = this->GetIndex(*it);
      if (i >= count.size())
        count.resize(i+1, 0);
      ++count[i];
    }
    ConvertCountsToWeights(count, t.size());
  }


  void
  Binning::ConvertCountsToWeights(const vector<unsigned int>& count, const unsigned int n)
  {
    fWeight.clear();
    const unsigned int nc = count.size();
    fWeight.resize(nc, 0);
    unsigned int nonzero = 0;
    for (size_t i = 0; i < nc; ++i)
      if (count[i]) {
        ++nonzero;
        const double invCount = 1. / count[i];
        fWeight[i] = invCount;
    }
    const double sum = double(n) / nonzero;
    for (std::vector<double>::iterator it = fWeight.begin(), end = fWeight.end();
         it != end; ++it)
      *it *= sum;
  }

}
