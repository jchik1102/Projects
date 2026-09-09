function results = test_openplc_modbus_smoke()
% verify the openplc runtime, modbus map, communication watchdog, automatic mode, batch start logic, and raw-water pump command

check_openplc_requirements();

fprintf('\n============================================================\n');
fprintf(' OPENPLC MODBUS SMOKE TEST\n');
fprintf('============================================================\n');

m = modbus('tcpip', '127.0.0.1', 5020, 'Timeout', 3);

cleanup = onCleanup(@() localSafeStop(m)); %#ok<NASGU>

localClearRequests(m);

write(m, 'holdingregs', 201, [4000 120 800 2 500]);

heartbeat = 1;

for k = 1:25
    heartbeat = localCycle( ...
        m, ...
        heartbeat, ...
        [700 100 700 0 0 300 0 4000], ...
        true);
end

localPulse(m, 104);

for k = 1:4
    heartbeat = localCycle( ...
        m, ...
        heartbeat, ...
        [700 100 700 0 0 300 0 4000], ...
        true);
end

localPulse(m, 101);

for k = 1:8
    heartbeat = localCycle( ...
        m, ...
        heartbeat, ...
        [700 100 700 0 0 300 0 4000], ...
        true);
end

mode = read(m, 'holdingregs', 306, 1);
state = read(m, 'holdingregs', 301, 1);
commHealthy = read(m, 'coils', 151, 1);
fillCommands = read(m, 'coils', 51, 2);

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
% write plant measurements, update heartbeat, and optionally

plantRegs(9) = heartbeat;

write(m, 'holdingregs', 1, plantRegs);

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


function localPulse(m, address)
% generate a momentary hmi request

write(m, 'coils', address, 1);
pause(0.15);

write(m, 'coils', address, 0);
pause(0.15);

end


function localClearRequests(m)
% clear only coil addresses that exist in the plc map

write(m, 'coils', 101, zeros(1, 6));

% C107-C109 arent mapped, leave them alone
write(m, 'coils', 110, zeros(1, 9));

write(m, 'coils', 251, zeros(1, 4));

end


function localSafeStop(m)
% attempt to stop the plc-controlled process before exiting

try
    % = system stop request
    write(m, 'coils', 102, 1);
    pause(0.15);

    write(m, 'coils', 102, 0);
catch
end

end