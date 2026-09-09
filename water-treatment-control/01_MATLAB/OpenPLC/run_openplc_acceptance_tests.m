function results = run_openplc_acceptance_tests()
%RUN_OPENPLC_ACCEPTANCE_TESTS
% Execute PLC-only integration tests using forced plant measurements and
% simulated equipment feedback through Modbus TCP.

check_openplc_requirements();

fprintf('\n============================================================\n');
fprintf(' OPENPLC ACCEPTANCE TESTS\n');
fprintf('============================================================\n');

% Connect to the OpenPLC Modbus TCP server.
m = modbus('tcpip', '127.0.0.1', 5020, 'Timeout', 3);

% Attempt a safe stop whenever this function exits.
cleanup = onCleanup(@() safeStop(m)); %#ok<NASGU>

% Initial HMI setpoints:
% HR201 = 400.0 kPa pressure setpoint
% HR202 = 1.20 mg/L concentration setpoint
% HR203 = 80.0% treatment-tank fill target
% HR204 = accelerated mixing-time setting
% HR205 = additional test setting
write(m, 'holdingregs', 201, [4000 120 800 2 500]);

% Clear only mapped coil ranges.
write(m, 'coils', 101, zeros(1, 6));  % C101-C106

% C107-C109 are intentionally unmapped.
write(m, 'coils', 110, zeros(1, 9));  % C110-C118

write(m, 'coils', 251, zeros(1, 4));  % C251-C254

heartbeat = 100;

names = strings(4, 1);
pass = false(4, 1);
details = strings(4, 1);

%% T1: Communication, automatic mode, and batch-state progression

% Establish healthy communications.
for k = 1:25
    heartbeat = cycle( ...
        m, ...
        heartbeat, ...
        [700 100 700 0 0 300 0 4000], ...
        true);
end

% C104 = Automatic-mode request.
pulse(m, 104);

% C101 = System-start request.
pulse(m, 101);

% Allow the state machine to enter Ready or Fill.
for k = 1:8
    heartbeat = cycle( ...
        m, ...
        heartbeat, ...
        [700 100 700 0 0 300 0 4000], ...
        true);
end

state1 = read(m, 'holdingregs', 301, 1);

% Force T-201 to the 80% batch-fill target.
for k = 1:5
    heartbeat = cycle( ...
        m, ...
        heartbeat, ...
        [650 800 700 350 0 300 0 4000], ...
        true);
end

state2 = read(m, 'holdingregs', 301, 1);

% Force concentration to 1.20 mg/L for over 20 seconds.
% Mixer feedback is mirrored from the PLC command.
for k = 1:210
    heartbeat = cycle( ...
        m, ...
        heartbeat, ...
        [650 800 700 0 0 300 120 4000], ...
        true);
end

state3 = read(m, 'holdingregs', 301, 1);

% Allow additional time for Mix/Verify/Transfer transitions.
for k = 1:30
    heartbeat = cycle( ...
        m, ...
        heartbeat, ...
        [650 800 700 0 0 300 120 4000], ...
        true);
end

state4 = read(m, 'holdingregs', 301, 1);

% Simulate transfer flow and mirror valve/pump feedback.
for k = 1:10
    heartbeat = cycle( ...
        m, ...
        heartbeat, ...
        [650 800 700 0 400 300 120 4000], ...
        true);
end

% Force the treatment tank to its transfer-complete level and increase the
% clean-water tank level.
for k = 1:5
    heartbeat = cycle( ...
        m, ...
        heartbeat, ...
        [650 100 750 0 400 300 120 4000], ...
        true);
end

batchCount = read(m, 'holdingregs', 302, 1);

names(1) = "Normal batch sequence";

pass(1) = ...
    any(state1 == [10 20]) && ...
    any(state2 == [30 40]) && ...
    any(state3 == [40 50 60]) && ...
    any(state4 == [50 60]) && ...
    batchCount >= 1;

details(1) = sprintf( ...
    'States %d -> %d -> %d -> %d; batch count %d', ...
    state1, ...
    state2, ...
    state3, ...
    state4, ...
    batchCount);

%% Reset and restart before the pressure tests

% C102 = Stop request.
pulse(m, 102);

% C103 = Reset request.
pulse(m, 103);

% C104 = Automatic-mode request.
pulse(m, 104);

% C101 = Start request.
pulse(m, 101);

for k = 1:12
    heartbeat = cycle( ...
        m, ...
        heartbeat, ...
        [700 100 700 0 0 300 0 4000], ...
        true);
end

%% T2: Pressure PI and booster staging request

% Force a high-demand, low-pressure condition:
% 60.0 L/s demand and 340.0 kPa measured pressure.
for k = 1:40
    heartbeat = cycle( ...
        m, ...
        heartbeat, ...
        [700 100 700 0 0 600 0 3400], ...
        true);
end

pressureOutput = read(m, 'holdingregs', 304, 1) / 10;
boosterCommands = read(m, 'coils', 54, 2);

names(2) = "Pressure PI and staging request";

pass(2) = ...
    pressureOutput > 95 && ...
    any(boosterCommands == 1);

details(2) = sprintf( ...
    'PI output %.1f%%; booster commands [%d %d]', ...
    pressureOutput, ...
    boosterCommands(1), ...
    boosterCommands(2));

%% T3: Lead-booster failure and standby takeover

% C251 = P-301A trip fault.
write(m, 'coils', 251, 1);

for k = 1:15
    heartbeat = cycle( ...
        m, ...
        heartbeat, ...
        [700 100 700 0 0 600 0 3400], ...
        true);
end

boosterAfterTrip = read(m, 'coils', 54, 2);
leadPump = read(m, 'holdingregs', 308, 1);

names(3) = "Lead booster failure takeover";

pass(3) = ...
    boosterAfterTrip(2) == 1 && ...
    leadPump == 2;

details(3) = sprintf( ...
    'Lead=%d; commands [%d %d]', ...
    leadPump, ...
    boosterAfterTrip(1), ...
    boosterAfterTrip(2));

% Clear the P-301A trip fault.
write(m, 'coils', 251, 0);

%% T4: Communication-loss safe state

% Continue writing the same heartbeat value so that the PLC detects stale
% communication. Do not increment heartbeat in this test.
for k = 1:25
    write( ...
        m, ...
        'holdingregs', ...
        1, ...
        [700 100 700 0 0 300 0 4000 heartbeat]);

    pause(0.11);
end

communicationFault = read(m, 'coils', 176, 1);
physicalCommands = read(m, 'coils', 51, 7);

names(4) = "Communication-loss safe state";

pass(4) = ...
    communicationFault == 1 && ...
    ~any(physicalCommands);

details(4) = sprintf( ...
    'Communication alarm=%d; active physical commands=%d', ...
    communicationFault, ...
    sum(physicalCommands));

%% Final results

results = table( ...
    names, ...
    pass, ...
    details, ...
    'VariableNames', {'Test', 'Pass', 'Details'});

disp(results);

assert( ...
    all(results.Pass), ...
    'One or more OpenPLC acceptance tests failed.');

fprintf('\nOPENPLC ACCEPTANCE TESTS: PASSED\n');

end


function heartbeat = cycle(m, heartbeat, registers, mirrorFeedback)
%CYCLE Perform one simulated plant/PLC communication cycle.

registers(9) = heartbeat;

write(m, 'holdingregs', 1, registers);

if mirrorFeedback
    % Read PLC output commands C51-C57.
    commands = read(m, 'coils', 51, 7);

    % Mirror commands into simulated plant feedback C1-C8.
    feedback = [
        commands(1:6), ...
        commands(7), ...
        ~logical(commands(7))
    ];

    write(m, 'coils', 1, double(feedback));
end

pause(0.11);

heartbeat = mod(heartbeat + 1, 65536);

end


function pulse(m, address)
%PULSE Generate a momentary Modbus coil command.

write(m, 'coils', address, 1);
pause(0.15);

write(m, 'coils', address, 0);
pause(0.15);

end


function safeStop(m)
%SAFESTOP Attempt to stop the process when the test ends.

try
    % C102 = System stop request.
    pulse(m, 102);
catch
    % Do not replace the original test error with a cleanup error.
end

end