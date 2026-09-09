function results = test_openplc_modbus_smoke()
%TEST_OPENPLC_MODBUS_SMOKE
% Verify the OpenPLC Runtime, Modbus map, communication watchdog,
% automatic mode, batch start logic, and raw-water pump command.

check_openplc_requirements();

fprintf('\n============================================================\n');
fprintf(' OPENPLC MODBUS SMOKE TEST\n');
fprintf('============================================================\n');

% Connect to the OpenPLC Modbus TCP server.
m = modbus('tcpip', '127.0.0.1', 5020, 'Timeout', 3);

% Attempt to issue a safe stop when this function ends or errors.
cleanup = onCleanup(@() localSafeStop(m)); %#ok<NASGU>

% Clear only mapped command and fault coils.
localClearRequests(m);

% Write initial HMI setpoints:
% HR201 = Pressure SP             = 4000 -> 400.0 kPa
% HR202 = Concentration SP        = 120  -> 1.20 mg/L
% HR203 = Batch fill SP           = 800  -> 80.0%
% HR204 = Mix time SP             = 2    -> accelerated test value
% HR205 = Additional test setting = 500
write(m, 'holdingregs', 201, [4000 120 800 2 500]);

heartbeat = 1;

% Establish healthy communications using safe plant values.
%
% Plant register vector:
% HR1 = T-101 level               (% x10)
% HR2 = T-201 level               (% x10)
% HR3 = T-301 level               (% x10)
% HR4 = Raw-water transfer flow   (L/s x10)
% HR5 = Treatment transfer flow   (L/s x10)
% HR6 = Distribution demand       (L/s x10)
% HR7 = Concentration             (mg/L x100)
% HR8 = Distribution pressure     (kPa x10)
% HR9 = Heartbeat
for k = 1:25
    heartbeat = localCycle( ...
        m, ...
        heartbeat, ...
        [700 100 700 0 0 300 0 4000], ...
        true);
end

% Request automatic mode using C104.
localPulse(m, 104);

for k = 1:4
    heartbeat = localCycle( ...
        m, ...
        heartbeat, ...
        [700 100 700 0 0 300 0 4000], ...
        true);
end

% Request system start using C101.
localPulse(m, 101);

for k = 1:8
    heartbeat = localCycle( ...
        m, ...
        heartbeat, ...
        [700 100 700 0 0 300 0 4000], ...
        true);
end

% Read PLC status.
mode = read(m, 'holdingregs', 306, 1);
state = read(m, 'holdingregs', 301, 1);
commHealthy = read(m, 'coils', 151, 1);
fillCommands = read(m, 'coils', 51, 2);

% Evaluate smoke-test requirements.
passes = [
    commHealthy == 1
    mode == 1
    any(fillCommands == 1)
    any(state == [10 20])
];

details = [
    "Heartbeat watchdog healthy"
    "Automatic mode active"
    "One raw-water duty pump requested"
    "Batch state reached Ready or Fill"
];

results = table( ...
    details, ...
    passes, ...
    'VariableNames', {'Check', 'Pass'});

disp(results);

assert( ...
    all(results.Pass), ...
    'OpenPLC smoke test failed. Review runtime logs and register addressing.');

fprintf('OPENPLC MODBUS SMOKE TEST: PASSED\n');

end


function heartbeat = localCycle(m, heartbeat, plantRegs, mirrorFeedback)
%LOCALCYCLE Write plant measurements, update heartbeat, and optionally
%mirror PLC equipment commands into simulated equipment feedback.

plantRegs(9) = heartbeat;

write(m, 'holdingregs', 1, plantRegs);

if mirrorFeedback
    % Read PLC output commands C51-C57.
    commands = read(m, 'coils', 51, 7);

    % Create simulated feedback C1-C8:
    % C1-C6 mirror motor/mixer commands.
    % C7 mirrors valve-open command.
    % C8 is valve-closed feedback.
    feedback = [
        commands(1:6), ...
        commands(7), ...
        ~logical(commands(7))
    ];

    write(m, 'coils', 1, double(feedback));
end

% Slightly longer than the 100 ms OpenPLC task period.
pause(0.11);

heartbeat = mod(heartbeat + 1, 65536);

end


function localPulse(m, address)
%LOCALPULSE Generate a momentary HMI request.

write(m, 'coils', address, 1);
pause(0.15);

write(m, 'coils', address, 0);
pause(0.15);

end


function localClearRequests(m)
%LOCALCLEARREQUESTS Clear only coil addresses that exist in the PLC map.

% C101-C106: system and mode requests.
write(m, 'coils', 101, zeros(1, 6));

% C107-C109 are intentionally unmapped.
%
% C110-C118: additional operator and maintenance requests.
write(m, 'coils', 110, zeros(1, 9));

% C251-C254: fault-injection commands.
write(m, 'coils', 251, zeros(1, 4));

end


function localSafeStop(m)
%LOCALSAFESTOP Attempt to stop the PLC-controlled process before exiting.

try
    % C102 = System stop request.
    write(m, 'coils', 102, 1);
    pause(0.15);

    write(m, 'coils', 102, 0);
catch
    % Do not hide the original test error if the cleanup write fails.
end

end