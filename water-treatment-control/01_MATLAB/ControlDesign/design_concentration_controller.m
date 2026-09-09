function D = design_concentration_controller(P, M, makePlots)
% tune and validate the dosing pi controller

if nargin < 1 || isempty(P), P = initialize_water_plant(false); end
if nargin < 2 || isempty(M), M = derive_plant_models(P, false); end
if nargin < 3, makePlots = false; end

[C, tuneInfo] = pidtune(M.concentration.loopPlant, 'PI', ...
    P.controller.concentration.targetCrossover_rad_s);
Ts = P.controller.concentration.sampleTime_s;
Cd = c2d(C, Ts, 'zoh');

L = minreal(C*M.concentration.loopPlant);
Tref = minreal(feedback(C*M.concentration.plant, M.concentration.sensor));
[Gm, Pm, Wcg, Wcp] = margin(L);
refInfo = stepinfo(Tref, 'SettlingTimeThreshold', 0.02);

Gd = c2d(M.concentration.plant, Ts, 'zoh');
Hd = c2d(M.concentration.sensor, Ts, 'zoh');
TrefDiscrete = minreal(feedback(Cd*Gd, Hd));

[Kpc, Kic, ~, ~] = piddata(C);
[Kp, Ki, ~, ~, Tsd] = piddata(Cd);

D.continuous = C;
D.discrete = Cd;
D.loop = L;
D.referenceClosedLoop = Tref;
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
D.antiWindup = P.controller.concentration.antiWindup;
D.outputMin_pct = P.controller.concentration.outputMin_pct;
D.outputMax_pct = P.controller.concentration.outputMax_pct;
D.metrics.gainMargin = Gm;
D.metrics.phaseMargin_deg = Pm;
D.metrics.phaseCrossover_rad_s = Wcg;
D.metrics.gainCrossover_rad_s = Wcp;
D.metrics.referenceRiseTime_s = refInfo.RiseTime;
D.metrics.referenceSettlingTime_s = refInfo.SettlingTime;
D.metrics.referenceOvershoot_pct = refInfo.Overshoot;
D.metrics.maximumDiscretePoleMagnitude = max(abs(pole(TrefDiscrete)));

if makePlots
    figure('Name','Concentration PI reference response');
    step(P.concentration.setpoint_mgL*Tref); grid on;
    title('Concentration PI reference response');
    figure('Name','Concentration PI loop margins');
    margin(L); grid on;
end
end
