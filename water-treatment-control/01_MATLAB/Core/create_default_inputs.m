function U = create_default_inputs(stopTime_s, assignToBase)
%CREATE_DEFAULT_INPUTS Create deterministic standalone-plant inputs.

if nargin < 1 || isempty(stopTime_s)
    stopTime_s = 120;
end
if nargin < 2
    assignToBase = false;
end

holdSignal = @(value) [0, value; stopTime_s, value];

U.raw_inflow = holdSignal(0);
U.demand_flow = holdSignal(30);

U.cmd_P101A = holdSignal(0);
U.cmd_P101B = holdSignal(0);
U.cmd_P201 = holdSignal(0);
U.cmd_P301A = holdSignal(0);
U.cmd_P301B = holdSignal(0);
U.cmd_DP201 = holdSignal(0);
U.cmd_M201 = holdSignal(0);
U.cmd_XV201 = holdSignal(0);

U.avail_P101A = holdSignal(1);
U.avail_P101B = holdSignal(1);
U.avail_P201 = holdSignal(1);
U.avail_P301A = holdSignal(1);
U.avail_P301B = holdSignal(1);

if assignToBase
    fields = fieldnames(U);
    for k = 1:numel(fields)
        assignin('base', fields{k}, U.(fields{k}));
    end
end
end
