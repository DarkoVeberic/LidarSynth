// $Id: GlobalMinimization.h 1852 2011-02-01 06:49:56Z darko $
#ifndef _lidar_GlobalMinimization_h_
#define _lidar_GlobalMinimization_h_
#include <utl/TemporaryModify.h>
#include <lidar/MeasurementModel.h>
#include <lidar/EstimateMeasurementModel.h>
#include <lidar/MaximumLikelihood.h>
#include <lidar/GlobalMinimizationResult.h>
#include <lidar/TrivialBinning.h>
#include <Minuit2/MnMinimize.h>
#include <Minuit2/FCNBase.h>
#include <Minuit2/FunctionMinimum.h>
#include <Minuit2/MnUserParameters.h>
#include <Minuit2/MnPrint.h>
#include <TError.h>
#include <iostream>


namespace lidar {

  template<class Binning>
  class GlobalMinimizationFunctor : public ROOT::Minuit2::FCNBase {
  public:
    GlobalMinimizationFunctor(const Binning& bin, const Trace& trace)
      : fBinning(bin), fTrace(trace) { }

    double
    operator()(const std::vector<double>& par)
      const
    {
      const MaximumLikelihood ml(
        MeasurementModel(
          CurrentModel(par[0], par[1], par[2]),
          PhotonCountModel(par[3])
        )
      );
      return ml.GetDeviance(fBinning, fTrace);
    }

    double Up() const { return 1; }

  private:
    const Binning& fBinning;
    const Trace& fTrace;
  };


  template<class Binning>
  GlobalMinimizationResult
  GlobalMinimization(const Binning& bin, const Trace& trace,
                     const MeasurementModel& mm)
  {
    using namespace ROOT::Minuit2;

    MnUserParameters params;
    const double f = 0.3;
    const CurrentModel& cm = mm.fCurrentModel;
    const PhotonCountModel& pcm = mm.fPhotonCountModel;
    params.Add("conversion", cm.fConversion, f*cm.fConversion);
    params.Add("baseline", cm.fBaseline, f*cm.fBaseline);
    params.Add("variance", cm.fVariance, /*f*cm.fVariance*/0);
    params.Add("deadFraction", pcm.fDeadFraction, f*pcm.fDeadFraction);
    params.SetLowerLimit("conversion", 0);
    params.Fix("variance");
    params.SetLowerLimit("baseline", 0);
    params.SetLowerLimit("deadFraction", 0);

    const GlobalMinimizationFunctor<Binning> func(bin, trace);

    utl::TemporaryModify<int> verbosity(gErrorIgnoreLevel, 1001);

    MnMinimize minimize(func, params);
    const FunctionMinimum fmin = minimize();

    if (!fmin.IsValid() || fmin.HesseFailed() || !fmin.HasValidCovariance())
      return GlobalMinimizationResult(false);

    const MnUserParameterState& us = fmin.UserState();

    const GlobalMinimizationResult res(
      true,
      MeasurementModel(
        CurrentModel(us.Value(0), us.Value(1), us.Value(2)),
        PhotonCountModel(us.Value(3))
      ),
      MeasurementModel(
        CurrentModel(us.Error(0), us.Error(1), us.Error(2)),
        PhotonCountModel(us.Error(3))
      )
    );

    return res;
  }


  template<class Binning>
  inline
  GlobalMinimizationResult
  GlobalMinimization(const Binning& bin, const Trace& trace)
  {
    const MeasurementModel mm = EstimateMeasurementModel(trace);
    return GlobalMinimization<Binning>(bin, trace, mm);
  }


  inline
  GlobalMinimizationResult
  GlobalMinimization(const Trace& trace)
  {
    return GlobalMinimization<>(TrivialBinning(trace), trace);
  }

}

#endif
