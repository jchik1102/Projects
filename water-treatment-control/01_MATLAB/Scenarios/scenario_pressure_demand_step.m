function S = scenario_pressure_demand_step()
%SCENARIO_PRESSURE_DEMAND_STEP Initial pressure-controller validation scenario.
P=initialize_water_plant(false);
S.name='Pressure demand step'; S.stopTime_s=60; S.stepTime_s=20;
S.initialDemand_Lps=P.pressure.nominalDemand_Lps;
S.finalDemand_Lps=P.pressure.highDemand_Lps;
S.acceptance.minimumPressure_kPa=P.pressure.minimumAllowed_kPa;
S.acceptance.recoveryTime_s=P.pressure.recoveryTimeLimit_s;
S.acceptance.finalBand_kPa=P.pressure.acceptanceBand_kPa;
end
