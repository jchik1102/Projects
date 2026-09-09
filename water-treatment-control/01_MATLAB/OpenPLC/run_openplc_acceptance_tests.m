function results = run_openplc_acceptance_tests()
% execute plc-only integration tests using forced plant measurements and simulated equipment feedback through modbus tcp

check_openplc_requirements();

fprintf('\n============================================================\n');
fprintf(' OPENPLC ACCEPTANCE TESTS\n');
fprintf('============================================================\n');

m = modbus('tcpip', '127.0.0.1', 5020, 'Timeout', 3);

cleanup = onCleanup(@() safeStop(m)); %#ok<NASGU>

write(m, 'holdingregs', 201, [4000 120 800 2 500]);

write(m, 'coils', 101, zeros(1, 6));  % C101-C106

% C107-C109 arent mapped, leave them alone
write(m, 'coils', 110, zeros(1, 9));  % C110-C118

write(m, 'coils', 251, zeros(1, 4));  % C251-C254

heartbeat = 100;

names = strings(4, 1);
pass = false(4, 1);
details = strings(4, 1);

%% t1: communication, automatic mode, and batch-state progression

for k = 1:25
    heartbeat = cycle( ...
        m, ...
        heartbeat, ...
        [700 100 700 0 0 300 0 4000], ...
        true);
end

% = automatic-mode request
pulse(m, 104);

% = system-start request
pulse(m, 101);

for k = 1:8
    heartbeat = cycle( ...
        m, ...
        heartbeat, ...
        [700 100 700 0 0 300 0 4000], ...
        true);
end

state1 = read(m, 'holdingregs', 301, 1);

for k = 1:5
    heartbeat = cycle( ...
        m, ...
        heartbeat, ...
        [650 800 700 350 0 300 0 4000], ...
        true);
end

state2 = read(m, 'holdingregs', 301, 1);

for k = 1:210
    heartbeat = cycle( ...
        m, ...
        heartbeat, ...
        [650 800 700 0 0 300 120 4000], ...
        true);
end

state3 = read(m, 'holdingregs', 301, 1);

for k = 1:30
    heartbeat = cycle( ...
        m, ...
        heartbeat, ...
        [650 800 700 0 0 300 120 4000], ...
        true);
end

state4 = read(m, 'holdingregs', 301, 1);

for k = 1:10
    heartbeat = cycle( ...
        m, ...
        heartbeat, ...
        [650 800 700 0 400 300 120 4000], ...
        true);
end

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

%% reset and restart before the pressure tests

% = stop request
pulse(m, 102);

% = reset request
pulse(m, 103);

% = automatic-mode request
pulse(m, 104);

% = start request
pulse(m, 101);

for k = 1:12
    heartbeat = cycle( ...
        m, ...
        heartbeat, ...
        [700 100 700 0 0 300 0 4000], ...
        true);
end

%% t2: pressure pi and booster staging request

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

%% t3: lead-booster failure and standby takeover

% = p-301a trip fault
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

write(m, 'coils', 251, 0);

%% t4: communication-loss safe state

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

%% final results

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
% perform one simulated plant/plc communication cycle

registers(9) = heartbeat;

write(m, 'holdingregs', 1, registers);

if mirrorFeedback
    commands = read(m, 'coils', 51, 7);

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
% generate a momentary modbus coil command

write(m, 'coils', address, 1);
pause(0.15);

write(m, 'coils', address, 0);
pause(0.15);

end


function safeStop(m)
% attempt to stop the process when the test ends

try
    % = system stop request
    pulse(m, 102);
catch
end

end