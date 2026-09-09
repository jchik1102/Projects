function S = scenario_concentration_setpoint()
%SCENARIO_CONCENTRATION_SETPOINT Initial dosing-loop validation scenario.
P=initialize_water_plant(false);
S.name='Concentration setpoint tracking'; S.stopTime_s=160;
S.setpoint_mgL=P.concentration.setpoint_mgL;
S.acceptance.low_mgL=P.concentration.acceptLow_mgL;
S.acceptance.high_mgL=P.concentration.acceptHigh_mgL;
S.acceptance.stableTime_s=P.concentration.stableTime_s;
S.acceptance.highHigh_mgL=P.concentration.highHigh_mgL;
end
