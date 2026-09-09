function D = design_pressure_controller(P, M, makePlots)
% tune and validate the pressure pi controller

if nargin < 1 || isempty(P), P = initialize_water_plant(false); end
if nargin < 2 || isempty(M), M = derive_plant_models(P, false); end
if nargin < 3, makePlots = false; end

[C, tuneInfo] = pidtune(M.pressure.loopPlant, 'PI', ...
    P.controller.pressure.targetCrossover_rad_s);
Ts = P.controller.pressure.sampleTime_s;
Cd = c2d(C, Ts, 'zoh');

L = minreal(C*M.pressure.loopPlant);
Tref = minreal(feedback(C*M.pressure.pump, M.pressure.sensor));
S = minreal(1/(1 + C*M.pressure.pump*M.pressure.sensor));
Tdemand = minreal(M.pressure.demand*S);

[Gm, Pm, Wcg, Wcp] = margin(L);
refInfo = stepinfo(Tref, 'SettlingTimeThreshold', 0.02);

t = (0:0.02:30).';
deltaDemand = (P.pressure.highDemand_Lps-P.pressure.nominalDemand_Lps)*ones(size(t));
deltaPressure = lsim(Tdemand, deltaDemand, t);
pressure = P.pressure.setpoint_kPa + deltaPressure;
recovery = recoveryTime(t, pressure, P.pressure.setpoint_kPa, ...
    P.pressure.acceptanceBand_kPa);

Gd = c2d(M.pressure.pump, Ts, 'zoh');
Hd = c2d(M.pressure.sensor, Ts, 'zoh');
TrefDiscrete = minreal(feedback(Cd*Gd, Hd));

[Kpc, Kic, ~, ~] = piddata(C);
[Kp, Ki, ~, ~, Tsd] = piddata(Cd);

D.continuous = C;
D.discrete = Cd;
D.loop = L;
D.referenceClosedLoop = Tref;
D.demandClosedLoop = Tdemand;
D.discreteReferenceClosedLoop = TrefDiscrete;
D.tuneInfo = tuneInfo;
D.continuousKp = Kpc;
D.continuousKi = Kic;
D.Kp = Kp;
D.Ki = Ki;
D.sampleTime_s = Ts;
D.KiTimesTs = Ki*Ts;
D.discreteKp = Kp;
D.discreteKi = Ki;
D.discreteSampleTime_s = Tsd;
D.integratorMethod = 'Forward Euler';
D.antiWindup = P.controller.pressure.antiWindup;
D.nominalCommand_pct = M.pressure.nominalTotalSpeed_pct;
D.deltaOutputMin_pct = P.controller.pressure.outputMin_pct-D.nominalCommand_pct;
D.deltaOutputMax_pct = P.controller.pressure.outputMax_pct-D.nominalCommand_pct;
D.outputMin_pct = P.controller.pressure.outputMin_pct;
D.outputMax_pct = P.controller.pressure.outputMax_pct;
D.metrics.gainMargin = Gm;
D.metrics.phaseMargin_deg = Pm;
D.metrics.phaseCrossover_rad_s = Wcg;
D.metrics.gainCrossover_rad_s = Wcp;
D.metrics.referenceRiseTime_s = refInfo.RiseTime;
D.metrics.referenceSettlingTime_s = refInfo.SettlingTime;
D.metrics.referenceOvershoot_pct = refInfo.Overshoot;
D.metrics.minimumPressureDemandStep_kPa = min(pressure);
D.metrics.demandRecoveryTime_s = recovery;
D.metrics.maximumDiscretePoleMagnitude = max(abs(pole(TrefDiscrete)));

if makePlots
    figure('Name','Pressure PI reference response');
    step(Tref); grid on; title('Pressure PI reference response');
    figure('Name','Pressure PI loop margins');
    margin(L); grid on;
end
end

function value = recoveryTime(t, y, target, band)
value = inf;
for k = 1:numel(t)
    if all(abs(y(k:end)-target) <= band)
        value = t(k);
        return;
    end
end
end
