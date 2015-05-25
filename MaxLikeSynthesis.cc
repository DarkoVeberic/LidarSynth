// $Id: MaxLikeSynthesis.cc 1835 2011-01-23 10:05:04Z darko $
#include <lidar/CurrentParameters.h>
#include <lidar/EstimateCurrentParameters.h>
#include <lidar/PhotonCountParameters.h>
#include <lidar/EstimatePhotonCountParameters.h>
#include <lidar/BaseMaximumLikelihood.h>
#include <lidar/FilterSaturation.h>
#include <lidar/TrivialBinning.h>
#include <lidar/FanBinning.h>
#include <lidar/InfinitelyFineBinning.h>
#include <lidar/CornerBinning.h>
#include <lidar/ReadFile.h>
#include <lidar/ShiftPhotonCount.h>
#include <TMinuit.h>
#include <iostream>

using namespace std;
using namespace lidar;


void
Test1()
{
  const CurrentParameters cp(20, 3.33928, 45.91, 22.955);
  const PhotonCountParameters pcp(20, 0.296280);

  const BaseMaximumLikelihood ml(cp, pcp);

  const Point d = { 0, 965, 5 };

  cout << "GetArgMinDeviance " << ml.GetArgMinDeviance(d) << endl;
}


void
Test2()
{
  Trace tTrace(20);
  ReadFile("../math/data/lidar-a.txt.bz2", tTrace);
  const Trace trace = FilterSaturation(tTrace);

  /*const CurrentParameters cp = { 20, 3.33928, 45.91, 22.955 };
  const PhotonCountParameters pcp = { 20, 0.296280 };*/
  const CurrentParameters cp = EstimateCurrentParameters(trace);
  const PhotonCountParameters pcp = EstimatePhotonCountParameters(trace);
  cerr << cp << '\n' << pcp << endl;
  const BaseMaximumLikelihood ml(cp, pcp);
  const CurrentModel& cm = ml.GetCurrentModel();

  for (vector<Point>::const_iterator it = trace.fTrace.begin(), end = trace.fTrace.end();
       it != end; ++it) {
    const double p = ml.GetArgMinDeviance(*it);
    cout << it->fIndex << ' '
         << it->fCurrent << ' '
         << it->fPhotonCount << ' '
         << p << ' '
         << cm.EstimatePhotonInput(it->fCurrent) << ' '
         << ml.Deviance(*it, p)
         << endl;
  }
}


void
Test3()
{
  const CurrentParameters cp(20, 3.33928, 45.91, 22.955);
  const PhotonCountParameters pcp(20, 0.296280);
  const BaseMaximumLikelihood ml(cp, pcp);
  const Point d = { 0, 1781, 67 };
  cerr << ml.GetArgMinDeviance(d) << endl;
}


namespace GlobalMinimization {

  const Trace* gTrace = 0;

  const TrivialBinning* gTrivialBinning = 0;
  const FanBinning* gFanBinning = 0;
  const InfinitelyFineBinning* gInfinitelyFineBinning = 0;
  const CornerBinning* gCornerBinning = 0;

  template<class Binning> const Binning& GetBinning();
  template<> const TrivialBinning& GetBinning<TrivialBinning>() { return *gTrivialBinning; }
  template<> const FanBinning& GetBinning<FanBinning>() { return *gFanBinning; }
  template<> const InfinitelyFineBinning& GetBinning<InfinitelyFineBinning>()
  { return *gInfinitelyFineBinning; }
  template<> const CornerBinning& GetBinning<CornerBinning>() { return *gCornerBinning; }

  template<class Binning> void SetBinning(const Binning& b);
  template<> void SetBinning<>(const TrivialBinning& b) { gTrivialBinning = &b; }
  template<> void SetBinning<>(const FanBinning& b) { gFanBinning = &b; }
  template<> void SetBinning<>(const InfinitelyFineBinning& b) { gInfinitelyFineBinning = &b; }
  template<> void SetBinning<>(const CornerBinning& b) { gCornerBinning = &b; }


  struct Result {
    Result(const bool converged) : fIsConverged(converged) { }

    bool fIsConverged;
    CurrentParameters fCurrentParameters;
    CurrentParameters fCurrentParametersError;
    PhotonCountParameters fPhotonCountParameters;
    PhotonCountParameters fPhotonCountParametersError;
  };


  ostream&
  operator<<(ostream& os, const Result& r)
  {
    return os
      << r.fCurrentParameters.fConversion << ' '
      << r.fCurrentParameters.fBaseline << ' '
      << r.fCurrentParameters.fVariance << ' '
      << r.fPhotonCountParameters.fDeadFraction;
  }


  template<class Binning>
  double
  TotalDeviance(const Result& res, const Binning& bin, const Trace& trace)
  {
    return BaseMaximumLikelihood(res.fCurrentParameters,
                                 res.fPhotonCountParameters).Deviance(bin, trace);
  }


  template<class Binning>
  void
  MinuitFunction(int& /*nPar*/, double* const /*grad*/, double& value,
                 double* const par, const int /*flag*/)
  {
    // par[]
    // 0: conversion
    // 1: baseline
    // 2: variance
    // 3: deadFraction
    const CurrentParameters cp(gTrace->fNShots, par[0], par[1], par[2]);
    const PhotonCountParameters pcp(gTrace->fNShots, par[3]);
    //cerr << "Got " << cp << ' ' << pcp << endl;
    const BaseMaximumLikelihood ml(cp, pcp);
    value = ml.DevianceShort(GetBinning<Binning>(), *gTrace);
    //cerr << "Deviance " << value << endl;
  }


  template<class Binning>
  Result
  Minimize(const Binning& bin, const Trace& trace,
           const CurrentParameters cp, const PhotonCountParameters pcp)
  {
    gTrace = &trace;
    SetBinning(bin);

    TMinuit minuit(4);
    int errFlag = 0;
    double argList[10];
    argList[0] = -1;
    minuit.mnexcm("SET PRINTOUT", argList, 1, errFlag);
    minuit.mnexcm("SET NOWARNINGS", argList, 0, errFlag);
    minuit.SetPrintLevel(-1);

    minuit.SetFCN(MinuitFunction<Binning>);
    argList[0] = 1;
    minuit.mnexcm("SET ERRORDEF", argList, 1, errFlag);

    const double f = 0.3;
    minuit.mnparm(0, "conversion", cp.fConversion, f*cp.fConversion, 0, 0, errFlag);
    minuit.mnparm(1, "baseline", cp.fBaseline, f*cp.fBaseline, 0, 0, errFlag);
    minuit.mnparm(2, "variance", cp.fVariance, f*cp.fVariance, 0, 0, errFlag);
    minuit.mnparm(3, "deadFraction", pcp.fDeadFraction, f*pcp.fDeadFraction, 0, 0, errFlag);

    minuit.FixParameter(2);

    argList[0] = 500;
    minuit.mnexcm("MINIMIZE", argList, 1, errFlag);
    if (errFlag) {
      cerr << "minuit minimize error: " << errFlag << endl;
      minuit.mnexcm("MINOS", argList, 0, errFlag);
      if (errFlag) {
        cerr << "  minuit minos error: " << errFlag << endl;
        return Result(false);
      }
    }

    Result res(true);
    res.fCurrentParameters.fNShots = cp.fNShots;
    res.fCurrentParametersError.fNShots = 0;
    minuit.GetParameter(0, res.fCurrentParameters.fConversion,
                           res.fCurrentParametersError.fConversion);
    minuit.GetParameter(1, res.fCurrentParameters.fBaseline,
                           res.fCurrentParametersError.fBaseline);
    minuit.GetParameter(2, res.fCurrentParameters.fVariance,
                           res.fCurrentParametersError.fVariance);
    res.fPhotonCountParameters.fNShots = pcp.fNShots;
    res.fPhotonCountParametersError.fNShots = 0;
    minuit.GetParameter(3, res.fPhotonCountParameters.fDeadFraction,
                           res.fPhotonCountParametersError.fDeadFraction);
    return res;
  }


  template<class Binning>
  inline
  Result
  Minimize(const Binning& bin, const Trace& trace)
  {
    const CurrentParameters cp = EstimateCurrentParameters(trace);
    const PhotonCountParameters pcp = EstimatePhotonCountParameters(trace);
    //cerr << "Initial " << cp << ' ' << pcp << endl;
    return Minimize(bin, trace, cp, pcp);
  }

}


void
Test4()
{
  Trace tTrace(20);
  ReadFile("../math/data/lidar-a.txt.bz2", tTrace);
  const Trace trace = FilterSaturation(tTrace);
  GlobalMinimization::Minimize(TrivialBinning(trace), trace);
}


void
Test5()
{
  Trace tTrace(20);
  ReadFile("../math/data/lidar-a.txt.bz2", tTrace);
  const Trace trace = FilterSaturation(tTrace);
  GlobalMinimization::Minimize(FanBinning(/*4096*20, 80,*/ 10000, trace), trace);
}


void
Test6()
{
  Trace tTrace(20);
  ReadFile("../math/data/lidar-a.txt.bz2", tTrace);
  const Trace trace = FilterSaturation(tTrace);
  GlobalMinimization::Minimize(InfinitelyFineBinning(trace), trace);
}


void
Test7()
{
  Trace tTrace(20);
  ReadFile("../math/data/lidar-a.txt.bz2", tTrace);
  const Trace trace = FilterSaturation(tTrace);
  const CurrentParameters cp = EstimateCurrentParameters(trace);
  const PhotonCountParameters pcp = EstimatePhotonCountParameters(trace);
  cout << "0.9 "
       << GlobalMinimization::Minimize(TrivialBinning(trace), trace, cp, pcp) << endl;
  const double ebStart = log(1000.);
  const double ebStop = log(200000.);
  const unsigned int ebN = 5000;
  const double ebStep = (ebStop - ebStart) / ebN;
  for (double eb = ebStart; eb <= ebStop; eb += ebStep) {
    const unsigned int b = exp(eb);
    cout << b
         << ' ' << GlobalMinimization::Minimize(FanBinning(b, trace), trace, cp, pcp) << endl;;
  }
  cout << "inf "
       << GlobalMinimization::Minimize(InfinitelyFineBinning(trace), trace, cp, pcp) << endl;
}


void
Test8()
{
  Trace tTrace(20);
  ReadFile("../math/data/lidar-a.txt.bz2", tTrace);
  const Trace trace = FilterSaturation(tTrace);
  unsigned int nBins = 0;
  for (unsigned int i = 66; i >= 1; --i) {
    const CornerBinning bin(trace, i);
    if (nBins != bin.GetNBins()) {
      nBins = bin.GetNBins();
      cout << bin.GetNBins() << ' ' << GlobalMinimization::Minimize(bin, trace) << endl;
    }
  }
}


void
Test9()
{
  Trace tTrace(20);
  ReadFile("../math/data/lidar-a.txt.bz2", tTrace);
  const Trace trace = FilterSaturation(tTrace);
  for (int offset = -30; offset <= 40; ++offset) {
    const Trace shifted = ShiftPhotonCount(trace, offset);
    const CornerBinning bin(shifted, 100);
    const GlobalMinimization::Result res = GlobalMinimization::Minimize(bin, shifted);
    const double dev = GlobalMinimization::TotalDeviance(res, bin, shifted);
    const unsigned int n = shifted.fTrace.size();
    cout << offset << ' ' << dev/n << ' '
         << res.fCurrentParameters.fConversion << ' '
         << res.fCurrentParameters.fBaseline << ' '
         << res.fCurrentParameters.fVariance << ' '
         << res.fPhotonCountParameters.fDeadFraction << endl;
  }
}


void
Test10()
{
  Trace tTrace(20);
  ReadFile("../math/data/lidar-a.txt.bz2", tTrace);
  const Trace trace = FilterSaturation(tTrace);
  const Trace shifted = ShiftPhotonCount(trace, 4);
  const CornerBinning bin(shifted, 100);
  const GlobalMinimization::Result res = GlobalMinimization::Minimize(bin, shifted);
  const double dev = GlobalMinimization::TotalDeviance(res, bin, shifted);
  const unsigned int n = shifted.fTrace.size();
  cout << 4 << ' ' << dev/n << ' '
       << res.fCurrentParameters.fConversion << ' '
       << res.fCurrentParameters.fBaseline << ' '
       << res.fCurrentParameters.fVariance << ' '
       << res.fPhotonCountParameters.fDeadFraction << endl;
}


int
main()
{
  //Test1();
  //Test2();
  //Test3();
  //Test4();
  //Test5();
  //Test6();
  //Test7();
  Test8();
  //Test9();
  //Test10();

  return 0;
}
