#include <utl/Random.h>
#include <utl/Sample.h>
#include <TRandom3.h>
#include <boost/lexical_cast.hpp>
#include <utility>
#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;
using namespace boost;
using namespace utl;


typedef pair<int, int> Pii;


template<typename A, typename B>
inline
ostream&
operator<<(ostream& os, const pair<A, B>& p)
{
  return os << p.first << ' ' << p.second;
}


template<typename T>
ostream&
operator<<(ostream& os, const vector<T>& v)
{
  typedef const vector<T> V;
  typedef typename V::const_iterator VCI;
  VCI it = v.begin();
  const VCI end = v.end();
  if (it != end) {
    os << *it;
    ++it;
  }
  for ( ; it != end; ++it)
    os << ' ' << *it;
  return os;
}


template<typename A, typename B>
inline
pair<A, B>&
operator+=(pair<A, B>& a, const pair<A, B>& b)
{
  a.first += b.first;
  a.second += b.second;
  return a;
}


vector<unsigned int>
Histogram(const vector<unsigned int>& v)
{
  vector<unsigned int> r;
  for (vector<unsigned int>::const_iterator it = v.begin(), end = v.end();
       it != end; ++it) {
    const unsigned int n = *it;
    if (n >= r.size())
      r.resize(n+1, 0);
    ++r[n];
  }
  return r; 
}


int
DeadTimeCount(const double n, const double deadFraction)
{
  if (n <= 0)
    return 0;
  const double start = gRandom->Rndm();
  const double stop = start + 1;
  int m = 0;
  double t = Random::Exponential(n);
  while (t <= stop) {
    if (start < t)
      ++m;
    t += deadFraction + Random::Exponential(n);
  }
  return m;
}


Pii
DeadTimeCount2(const double mean, const double deadFraction)
{
  if (mean <= 0)
    return make_pair(0, 0);
  const double tstart = gRandom->Rndm();
  const double tstop = tstart + 1;
  int n = 0;
  int m = 0;
  const double meanWait = 1. / mean;
  double t2 = 0;
  for (;;) {
    const double t1 = t2 + Random::Exponential(meanWait);
    if (t1 > tstop)
      break;
    t2 = t1 + deadFraction;
    if (t1 > tstart) {
      ++m;
      ++n;
    }
    if (t2 > tstart) {
      const double dt = min(tstop, t2) - max(tstart, t1);
      n += Random::Poisson(mean * dt);
    }
  }
  return make_pair(m, n);
}


Pii
DeadTimeCount3(const double mean, const double deadFraction)
{
  if (mean <= 0)
    return make_pair(0, 0);

  int n = 0;
  int m = 0;
  const double meanWait = 1. / mean;
  double t = -Random::Exponential(meanWait) + deadFraction;
  if (0 < t)
    t = 0;
  else if (t <= 1) {
    n += Random::Poisson(mean * t);
    t += Random::Exponential(meanWait);
  } else {
    n += Random::Poisson(mean);
    return make_pair(m, n);
  }
  while (t <= 1) {
    ++m;
    t += deadFraction;
    if (t <= 1)
      n += Random::Poisson(mean * deadFraction);
    else {
      n += Random::Poisson(mean * (1 - (t - deadFraction)));
      break;
    }
    t += Random::Exponential(meanWait);
  }
  return make_pair(m, n);
}


void
TraceDeadTimeCount(const double rate, const double deadFraction,
                   vector<unsigned int>& count, vector<unsigned int>& photon)
{
  const unsigned int tMax = count.size();
  const double meanWait = 1. / rate;
  // start with an event far from dead zone and exponentially from zero
  double t = Random::Exponential(meanWait);
  double tKill = -2 * deadFraction;
  while (t < tMax) {
    const unsigned int i = (unsigned int)(t);
    ++photon[i];
    if (t - tKill > deadFraction) {
      ++count[i];
      tKill = t;
    }
    t += Random::Exponential(meanWait);
  }
}


void
FastTraceDeadTimeCount(const double rate, const double deadFraction,
                       vector<unsigned int>& count, vector<unsigned int>& photon)
{
  const unsigned int tMax = count.size();
  const double meanWait = 1. / rate;
  // start with an event far from dead zone and exponentially from zero
  double t = Random::Exponential(meanWait);
  while (t < tMax) {
    const unsigned int i = (unsigned int)(t);
    ++count[i];
    ++photon[i];
    const double tt = t + deadFraction;
    const unsigned int i1 = i + 1;
    if (tt < i1)
      photon[i] += Random::Poisson(rate * deadFraction);
    else {
      photon[i] += Random::Poisson(rate * (i1 - t));
      photon[i+1] += Random::Poisson(rate * (tt - i1));
    }
    t = tt + Random::Exponential(meanWait);
  }
}


inline
double
LidarReturn(const double x)
{
  return 70155.7 * pow(1 - exp(-x/130), 3) / (x*x) * exp(-x/1160.66);
}


inline
double
PhotonToAnalog(const double photons)
{
  return 45.873 + 5.24796 * photons + Random::Normal(0, 6.7);
}


void
TraceSimulation(const unsigned int nShots)
{
  const double p0 = 8652.38;
  const double deadFraction = 0.297591;
  const unsigned int traceLength = 16000;

  for (unsigned int i = 1; i <= traceLength; ++i) {
    const double photon = p0 * LidarReturn(i);
    Pii count(0, 0);
    double analog = 0;
    for (unsigned int j = 0; j < nShots; ++j) {
      const Pii c = DeadTimeCount2(photon, deadFraction);
      count += c;
      analog += PhotonToAnalog(c.second);
    }
    cout << analog << ' ' << count << ' ' << nShots * photon << '\n';
  }
}


template<class DeadTimeCount>
void
CountingVariance(DeadTimeCount& DTC, const unsigned int nShots)
{
  const unsigned int n = 10000;
  const double minRateExp = -3;
  const double maxRateExp = 5;
  const double rateExpStep = 0.01;
  const double deadFraction = 0.3456;
  //const double deadFraction = 0.297591;
  //const double deadFraction = 0.1234;
  //const double deadFraction = 0.0678324;

  vector<unsigned int> count(n);
  vector<unsigned int> photon(n);
  for (double rateExp = minRateExp; rateExp <= maxRateExp; rateExp += rateExpStep) {
    const double photonRate = pow(10., rateExp);
    for (unsigned int i = 0; i < n; ++i) {
      Pii cp(0, 0);
      for (unsigned int j = 0; j < nShots; ++j)
        cp += DTC(photonRate, deadFraction);
      count[i] = cp.first;
      photon[i] = cp.second;
    }
    const double cMean = Sample::Mean(count.begin(), count.end());
    const double pMean = Sample::Mean(photon.begin(), photon.end());
    cout
      << photonRate << ' '
      << cMean << ' '
      << Sample::Variance(count.begin(), count.end(), cMean) << ' '
      << pMean << ' '
      << Sample::Variance(photon.begin(), photon.end(), pMean)
      << '\n';
  }
}


template<class DeadTimeCount>
void
CountingVarianceDeadTime(DeadTimeCount& DTC, const unsigned int nShots)
{
  const unsigned int n = 10000;
  const double photonRate = 1e2;

  vector<double> dt;
  for (double de = -4; de < -1.5; de += 0.05)
    dt.push_back(pow(10., de));
  for (double d = pow(10., -1.5); d <= 1; d += d/200)
    dt.push_back(d);

  vector<unsigned int> count(n);
  vector<unsigned int> photon(n);
  for (vector<double>::const_iterator dIt = dt.begin(), end = dt.end();
       dIt != end; ++dIt) {
    for (unsigned int i = 0; i < n; ++i) {
      Pii cp(0, 0);
      for (unsigned int j = 0; j < nShots; ++j)
        cp += DTC(photonRate, *dIt);
      count[i] = cp.first;
      photon[i] = cp.second;
    }
    const double cMean = Sample::Mean(count.begin(), count.end());
    const double pMean = Sample::Mean(photon.begin(), photon.end());
    cout
      << *dIt << ' '
      << cMean << ' '
      << Sample::Variance(count.begin(), count.end(), cMean) << ' '
      << pMean << ' '
      << Sample::Variance(photon.begin(), photon.end(), pMean)
      << '\n';
  }
}


template<class DeadTimeCount>
void
CountingVarianceRateDeadTime(DeadTimeCount& DTC, const unsigned int nShots)
{
  const unsigned int n = 10000;

  vector<double> pr;
  for (double pe = -3; pe <= 5; pe += 0.01)
    pr.push_back(pow(10., pe));

  vector<double> dt;
  for (double de = -4; de < -1.5; de += 0.05)
    dt.push_back(pow(10., de));
  for (double d = pow(10., -1.5); d <= 1; d += d/200)
    dt.push_back(d);

  vector<unsigned int> count(n);
  vector<unsigned int> photon(n);
  for (vector<double>::const_iterator pIt = pr.begin(), end = pr.end();
       pIt != end; ++pIt) {
    for (vector<double>::const_iterator dIt = dt.begin(), end = dt.end();
         dIt != end; ++dIt) {
      for (unsigned int i = 0; i < n; ++i) {
        Pii cp(0, 0);
        for (unsigned int j = 0; j < nShots; ++j)
          cp += DTC(*pIt, *dIt);
        count[i] = cp.first;
        photon[i] = cp.second;
      }
      const double cMean = Sample::Mean(count.begin(), count.end());
      const double pMean = Sample::Mean(photon.begin(), photon.end());
      cout
        << *pIt << ' '
        << *dIt << ' '
        << cMean << ' '
        << Sample::Variance(count.begin(), count.end(), cMean) << ' '
        << pMean << ' '
        << Sample::Variance(photon.begin(), photon.end(), pMean)
        << '\n';
    }
  }
}


void
TraceCountingVarianceDeadTime(const unsigned int nShots)
{
  const double photonRate = 1000;
  const unsigned int n =
    photonRate > 0.5 ? 50000000 / photonRate : 100000000;

  vector<double> dt;
  for (double de = -4; de < -1.5; de += 0.05)
    dt.push_back(pow(10., de));
  for (double d = pow(10., -1.5); d <= 1; d += d/200)
    dt.push_back(d);

  vector<unsigned int> count(n);
  vector<unsigned int> photon(n);
  for (vector<double>::const_iterator dIt = dt.begin(), end = dt.end();
       dIt != end; ++dIt) {
    count.assign(n, 0);
    photon.assign(n, 0);
    for (unsigned int i = 0; i < nShots; ++i)
      FastTraceDeadTimeCount(photonRate, *dIt, count, photon);
    const double cMean = Sample::Mean(count.begin(), count.end());
    const double cVar = Sample::Variance(count.begin(), count.end(), cMean);
    const double pMean = Sample::Mean(photon.begin(), photon.end());
    const double pVar = Sample::Variance(photon.begin(), photon.end(), pMean);
    cout
      << *dIt << ' ' << cMean << ' ' << cVar << ' ' << pMean << ' ' << pVar << ' '
      << Sample::Autocorrelation(count.begin(), count.end(), 20, cMean, cVar) << ' '
      << Sample::Autocorrelation(photon.begin(), photon.end(), 20, pMean, pVar) << ' '
      << endl;
  }
}


int
main(int argc, char* argv[])
{
  delete gRandom;
  gRandom = new TRandom3(0);

  gRandom->SetSeed(13);

  const unsigned int nShots =
    argc > 1 ? lexical_cast<unsigned int>(argv[1]) : 1;

  //TraceSimulation(nShots);
  //CountingVariance(DeadTimeCount2<MTRandom>, nShots);
  //CountingVarianceDeadTime(DeadTimeCount3<MTRandom>, nShots);
  //CountingVarianceRateDeadTime(DeadTimeCount2<MTRandom>, nShots);
  TraceCountingVarianceDeadTime(nShots);

  return 0;
}
