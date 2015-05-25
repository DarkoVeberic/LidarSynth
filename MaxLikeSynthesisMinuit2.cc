// $Id: MaxLikeSynthesisMinuit2.cc 1935 2011-08-31 21:39:52Z darko $
#include <lidar/MeasurementModel.h>
#include <lidar/EstimateMeasurementModel.h>
#include <lidar/MaximumLikelihood.h>
#include <lidar/FilterSaturation.h>
#include <lidar/TrivialBinning.h>
#include <lidar/FanBinning.h>
#include <lidar/InfinitelyFineBinning.h>
#include <lidar/CornerBinning.h>
#include <lidar/GlobalMinimization.h>
#include <lidar/TotalDeviance.h>
#include <lidar/CombinedReconstruction.h>
#include <lidar/ReadFile.h>
#include <lidar/ShiftPhotonCount.h>
#include <utl/io/ZFStream.h>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;
using namespace lidar;


void
Test1()
{
  /*const MeasurementModel mm(
    CurrentModel(3.10846, 918.377, 469.725),
    PhotonCountModel(0.0148088)
  );
  const Point d = { 0, 1070, 0 };*/
  
  const MeasurementModel mm(
    CurrentModel(1.52578, 915.133, 1013.12),
    PhotonCountModel(0.0131561)
  );
  const Point d = { 0, 1287, 3 };

  const pair<double, double> ap = mm.GetApproximateArgMinDeviance(d);
  cout << "GetApproximateArgMinDeviance " << ap.first << " (" << sqrt(ap.first) << ") "
       << ap.second << " (" << sqrt(ap.second) << ')' << endl;
  const pair<double, double> pd = mm.GetArgMinDeviance(d);
  cout << "GetArgMinDeviance " << pd.first << ' ' << pd.second << endl;
  cout << "IsMinimum " << mm.IsMinimum(d, pd.first) << endl;
}


void
Test2()
{
  const int offset = -29;
  Trace tTrace(20);
  ReadFile("../math/data/lidar-a.txt.bz2", tTrace);
  const Trace trace = ShiftPhotonCount(FilterSaturation(tTrace), offset);

  //const MeasurementModel mm = EstimateMeasurementModel(trace);
  const MeasurementModel mm(
    CurrentModel(1.52578, 915.133, 1013.12),
    PhotonCountModel(0.0131561)
  );
  cout << mm << endl;

  double dev = 0;
  for (vector<Point>::const_iterator it = trace.fTrace.begin(), end = trace.fTrace.end();
       it != end; ++it) {
    const pair<double, double> app = mm.GetApproximateArgMinDeviance(*it);
    cout << *it << ' '
         << app.first << ' ' << app.second << ' '
         << flush;
    const pair<double, double> pd = mm.GetArgMinDeviance(*it);
    const double p = pd.first;
    dev += pd.second;
    cout << p << ' ' << pd.second << ' ';
    if (!it->fPhotonCount) {
      const pair<double, double> cr = mm.fCurrentModel.GetD12(*it, p);
      const pair<double, double> pr = mm.fPhotonCountModel.GetD12(*it, p);
      cout << '[' << cr.first << ' ' << cr.second << ' ' << pr.first << ' ' << pr.second << "] ";
    }
    cout << mm.GetDRatio(*it, p) << ' '
         << mm.IsMinimum(*it, p)
         << '\n';
  }
  cout << "total deviance " << dev << endl;
}


void
Test3()
{
  const int offset = -29;
  Trace tTrace(20);
  ReadFile("../math/data/lidar-a.txt.bz2", tTrace);
  const Trace trace = ShiftPhotonCount(FilterSaturation(tTrace), offset);
  const MeasurementModel mm = EstimateMeasurementModel(trace);
  cout << "init " << mm << endl;
  const GlobalMinimizationResult res = GlobalMinimization<>(TrivialBinning(trace), trace, mm);
  if (res.fIsConverged) {
    const double dev = TotalDeviance(res, trace);
    const unsigned int n = trace.fTrace.size();
    cout << offset << ' ' << dev/n << ' ' << res << endl;
  } else
    cout << "did not converge!" << endl;
}


void
TestRelativeDelay()
{
  Trace tTrace(20);
  ReadFile("../math/data/lidar-a.txt.bz2", tTrace);
  const Trace trace = FilterSaturation(tTrace);
  for (int offset = -100; offset <= 100; ++offset) {
    const Trace shifted = ShiftPhotonCount(trace, offset);
    const GlobalMinimizationResult res = GlobalMinimization(shifted);
    const double dev = TotalDeviance(res, shifted);
    const unsigned int n = shifted.fTrace.size();
    cout << offset << ' ' << dev/n << ' ' << res << endl;
  }
}


void
TestCornerBinning()
{
  Trace tTrace(20);
  ReadFile("../math/data/lidar-a.txt.bz2", tTrace);
  const Trace trace = ShiftPhotonCount(FilterSaturation(tTrace), 4);
  const CornerBinning bin(trace, 100);
  const unsigned int c = 81000;
  const unsigned int pc = 1;
  for (unsigned int c = 0; c < 81000; c += 100) {
    const Point p = { 0, c, pc };
    cout << c << ' ' << pc << ' ' << bin.GetIndex(p) << endl;
  }
  for (unsigned int pc = 0; pc < 80; ++pc) {
    const Point p = { 0, c, pc };
    cout << c << ' ' << pc << ' ' << bin.GetIndex(p) << endl;
  }
}


void
TestCornerBinning2()
{
  Trace tTrace(20);
  ReadFile("../math/data/lidar-a.txt.bz2", tTrace);
  const Trace trace = ShiftPhotonCount(FilterSaturation(tTrace), 4);
  const CornerBinning bin(trace, 10);
  typedef vector<Point> RV;
  const RV& t = trace.fTrace;
  for (RV::const_iterator it = t.begin(), end = t.end(); it != end; ++it)
    cout << it->fCurrent << ' ' << it->fPhotonCount << ' ' << bin.GetIndex(*it) << endl;
}


void
TestCornerBinning3()
{
  Trace tTrace(20);
  ReadFile("../math/data/lidar-a.txt.bz2", tTrace);
  const Trace trace = ShiftPhotonCount(FilterSaturation(tTrace), 4);
  const CornerBinning bin(trace, 10);
  bin.Dump();
}


void
TestRelativeDelayWithCornerBinning()
{
  Trace tTrace(20);
  ReadFile("../math/data/lidar-a.txt.bz2", tTrace);
  const Trace trace = FilterSaturation(tTrace);
  for (int offset = -30; offset <= 40; ++offset) {
    const Trace shifted = ShiftPhotonCount(trace, offset);
    const CornerBinning bin(shifted, 100);
    const GlobalMinimizationResult res = GlobalMinimization(bin, shifted);
    const double dev = TotalDeviance(res, bin, shifted);
    const unsigned int n = shifted.fTrace.size();
    cout << offset << ' ' << dev/n << ' ' << res
         << endl;
  }
}


void
TestEstimate()
{
  Trace tTrace(20);
  ReadFile("../math/data/lidar-a.txt.bz2", tTrace);
  const Trace trace = ShiftPhotonCount(FilterSaturation(tTrace), 4);
  const PhotonCountModel pcm = EstimatePhotonCountModel(trace);
  const CurrentModel cm1 = EstimateCurrentModel(trace);
  const CurrentModel cm2 = EstimateCurrentModel(trace, pcm);
  cout << "cm1:\n" << cm1 << "\ncm2:\n" << cm2 << endl;
}


void
TestBinning()
{
  Trace tTrace(20);
  //ReadFile("../math/data/lidar-a.txt.bz2", tTrace);
  ReadFile("../math/data/lidar-a-a_02000.txt.bz2", tTrace);
  const Trace trace = ShiftPhotonCount(FilterSaturation(tTrace), 4);
  const CurrentModel cm1 = EstimateCurrentModel(trace);
  const PhotonCountModel pcm = EstimatePhotonCountModel(trace);
  const CurrentModel cm2 = EstimateCurrentModel(trace, pcm);
  const MeasurementModel mm(cm2, pcm);
  ofstream ofs;
  ofs.open("binning_dependence-est.txt");
  ofs << "0.3 "
      << GlobalMinimizationResult(true, MeasurementModel(cm1, pcm)) << "\n"
         "0.4 "
      << GlobalMinimizationResult(true, mm) << endl;
  ofs.close();
  ofs.open("binning_dependence-triv.txt");
  {
    const TrivialBinning triv(trace);
    ofs << "0.6 "
        << GlobalMinimization(triv, trace, mm) << ' '
        << triv.GetNorm()
        << endl;
  }
  ofs.close();
  ofs.open("binning_dependence-fan.txt");
  {
    ofs << "# 1:nBins 2:converged 3:nShots 4:alpha 5:beta 6:gamma 7:ignore 8:alphaErr 9:betaErr 10:gammaErr "
           "11:nShots 12:delta 13:ignore 14:deltaErr 15:nEmptyBins 16:nNonzeroBins 17:norm "
           "18:minWeight 19:minNonzeroWeight 20:maxWeight 21:meanWeight 22:meanNonzeroWeight\n";
    const double ebStart = log(1.);
    const double ebStop = log(200000.);
    const unsigned int ebN = 60;
    const double ebStep = (ebStop - ebStart) / ebN;
    unsigned int nBins = 0;
    int bb = exp(ebStart) - 1;
    for (double eb = ebStart; eb <= ebStop; eb += ebStep) {
      int b = exp(eb);
      if (b != bb) {
        const FanBinning fan(b, trace);
        if (nBins != fan.GetNBins()) {
          ofs << fan.GetNBins() << ' '
              << GlobalMinimization(fan, trace, mm) << ' '
              << fan.GetNEmptyBins() << ' '
              << fan.GetNNonemptyBins() << ' '
              << fan.GetNorm(trace) << ' '
              << fan.GetMinWeight() << ' ' << fan.GetMinNonzeroWeight() << ' ' << fan.GetMaxWeight() << ' '
              << fan.GetMeanWeight() << ' '
              << fan.GetMeanNonzeroWeight()
              << endl;
          cerr << '.' << flush;
          nBins = fan.GetNBins();
          bb = b;
        }
      }
    }
  }
  ofs.close();
  cerr << endl;
  ofs.open("binning_dependence-corner.txt");
  {
    unsigned int nNonemptyBins = 0;
    for (double b = 0.2; b < 10000; b *= 1.3) {
      const CornerBinning corner(trace, b);
      if (nNonemptyBins != corner.GetNNonemptyBins()) {
        ofs << corner.GetNBins() << ' '
            << GlobalMinimization(corner, trace, mm) << ' '
            << corner.GetNEmptyBins() << ' '
            << corner.GetNNonemptyBins() << ' '
            << corner.GetNorm(trace) << ' '
            << corner.GetMinWeight() << ' ' << corner.GetMinNonzeroWeight() << ' ' << corner.GetMaxWeight() << ' '
            << corner.GetMeanWeight() << ' '
            << corner.GetMeanNonzeroWeight()
            << endl;
        cerr << '.' << flush;
        nNonemptyBins = corner.GetNNonemptyBins();
      }
    }
  }
  ofs.close();
  cerr << endl;
  ofs.open("binning_dependence-inf.txt");
  {
    const InfinitelyFineBinning inf(trace);
    ofs << inf.GetNNonemptyBins() << ' '
        << GlobalMinimization(inf, trace, mm) << ' '
        << inf.GetNorm(trace)
        << endl;
  }
  ofs.close();
}


void
TestReconstruction()
{
  Trace t(20);
  ReadFile("../math/data/lidar-a.txt.bz2", t);
  const Trace tt = ShiftPhotonCount(FilterSaturation(t), 4);
  const GlobalMinimizationResult r = GlobalMinimization(tt);
  cerr << "res:\n" << r << endl;
  const PhotonTrace p = MaximumLikelihood(r).ReconstructPhotons(tt);
  const CombinedReconstructionTrace c = CombinedReconstruction(tt, p, r);
  copy(c.begin(), c.end(), ostream_iterator<CombinedReconstructionPoint>(cout, "\n"));
}


void
TestSpeed()
{
  Trace t(20);
  ReadFile("../math/data/lidar-a.txt.bz2", t);
  for (int i = 0; i < 100; ++i) {
    const Trace tt = ShiftPhotonCount(FilterSaturation(t), 4);
    const GlobalMinimizationResult r = GlobalMinimization(tt);
    //cerr << "res:\n" << r << endl;
    const PhotonTrace p = MaximumLikelihood(r).ReconstructPhotons(tt);
    const CombinedReconstructionTrace c = CombinedReconstruction(tt, p, r);
    //copy(c.begin(), c.end(), ostream_iterator<CombinedReconstructionPoint>(cout, "\n"));
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
  const GlobalMinimizationResult res = GlobalMinimization(bin, shifted);
  const double dev = TotalDeviance(res, bin, shifted);
  const unsigned int n = shifted.fTrace.size();
  cout << 4 << ' ' << dev/n << ' '
       << res.fMeasurementModel << endl;
}


void
TestTimeStability()
{
  using namespace utl::io;
  ZIFStream file("../math/data/all_traces/"
                 //"RD-20090701-001102-NShots20-NEvents420.txt.bz2"
                 "RD-20090701-010245-NShots20-NEvents420.txt.bz2"
                );

  unsigned int nTraces = 0;
  vector<unsigned int> current;
  vector<unsigned int> photonCount;
  string line;
  istringstream is;
  while (file) {
    if (!getline(file, line))
      break;
    else
      cerr << line << ' ';
    if (!getline(file, line)) {
      cerr << "Cannot read current line!" << endl;
      break;
    } else {
      is.clear();
      is.str(line);
      current.clear();
      copy(istream_iterator<unsigned int>(is), istream_iterator<unsigned int>(),
           back_inserter(current));
    }
    if (!getline(file, line)) {
      cerr << "Cannot read photon counting line!" << endl;
      break;
    } else {
      is.clear();
      is.str(line);
      photonCount.clear();
      copy(istream_iterator<unsigned int>(is), istream_iterator<unsigned int>(),
           back_inserter(photonCount));
    }
    if (current.empty() || photonCount.empty() || current.size() != photonCount.size()) {
      cerr << "Nothing read!" << endl;
      break;
    }

    cerr << current.size() << ' ' << photonCount.size() << endl;

    ++nTraces;

    Trace tt(20);
    for (int i = 0, n = current.size(); i < n; ++i) {
      const Point p = { i, current[i], photonCount[i] };
      tt.fTrace.push_back(p);
    }

    const Trace trace = ShiftPhotonCount(FilterSaturation(tt), 4);
    const CurrentModel cm1 = EstimateCurrentModel(trace);
    const PhotonCountModel pcm = EstimatePhotonCountModel(trace);
    const CurrentModel cm2 = EstimateCurrentModel(trace, pcm);
    const MeasurementModel mm(cm2, pcm);

    cout << nTraces << "  "
         << GlobalMinimizationResult(true, MeasurementModel(cm1, pcm)) << '\t'
         << GlobalMinimizationResult(true, mm) << '\t';

    const TrivialBinning triv(trace);
    cout << GlobalMinimization(triv, trace, mm) << '\t';

    const InfinitelyFineBinning inf(trace);
    cout << GlobalMinimization(inf, trace, mm) << endl;
  }
}


int
main()
{
  //Test1();
  //Test2();
  //Test3();
  //TestRelativeDelay();
  //TestCornerBinning();
  //TestCornerBinning2();
  //TestCornerBinning3();
  //TestEstimate();
  //TestBinning();
  //TestReconstruction();
  TestSpeed();
  //TestTimeStability();

  return 0;
}
