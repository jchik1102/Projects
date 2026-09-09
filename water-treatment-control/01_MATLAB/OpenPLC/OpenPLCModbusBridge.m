classdef OpenPLCModbusBridge < matlab.System
    % real-time modbus bridge for the simulink plant

    properties (Nontunable)
        Host = '127.0.0.1'
        Port = 5020
        Timeout_s = 1
        SampleTime_s = 0.10
    end

    properties (Access = private, Transient)
        Client = []
    end

    properties (DiscreteState)
        Heartbeat
        LastAIT201
        ExchangeCount
        ReconnectCountdown
        WarningIssued
    end

    methods (Static)
        function [size1, size2, size3, size4, size5] = outputPortSizes()
            % port shapes need to match the row vectors from stepImpl
            size1 = [1 6];
            size2 = [1 7];
            size3 = [1 4];
            size4 = [1 1];
            size5 = [1 48];
        end
    end

    methods (Access = protected)
        function setupImpl(obj, ~, ~)
            obj.resetStates();
            obj.tryConnect();
        end

        function [analogCommands, digitalCommands, faultInputs, commHealthy, diagnostics] = ...
                stepImpl(obj, measurements, feedback)

            analogCommands = zeros(1, 6);
            digitalCommands = zeros(1, 7);
            faultInputs = zeros(1, 4);
            commHealthy = 0;
            diagnostics = zeros(1, 48);

            if isempty(obj.Client)
                if obj.ReconnectCountdown > 0
                    obj.ReconnectCountdown = obj.ReconnectCountdown - 1;
                    return;
                end
                obj.tryConnect();
                if isempty(obj.Client)
                    return;
                end
            end

            try
                analogRaw = read(obj.Client, 'holdingregs', 101, 6);
                digitalRaw = read(obj.Client, 'coils', 51, 7);
                faultRaw = read(obj.Client, 'coils', 251, 4);
                diagnosticHoldingRaw = read( ...
                    obj.Client, 'holdingregs', 301, 17);
                diagnosticCoilRaw = read(obj.Client, 'coils', 151, 31);

                analogCommands = reshape(double(analogRaw), 1, 6) / 10;
                digitalCommands = reshape(double(digitalRaw), 1, 7);
                faultInputs = reshape(double(faultRaw), 1, 4);
                diagnostics = [ ...
                    reshape(double(diagnosticHoldingRaw), 1, 17), ...
                    reshape(double(diagnosticCoilRaw), 1, 31)];

                measurements = reshape(double(measurements), 1, 8);
                feedback = reshape(double(feedback), 1, 7);

                % freeze the sensor reading only; the actual concentration still changes
                if faultInputs(2) == 0
                    obj.LastAIT201 = measurements(7);
                end
                measurements(7) = obj.LastAIT201;

                % hold the heartbeat to trigger the plc watchdog
                if faultInputs(4) == 0
                    obj.Heartbeat = mod(obj.Heartbeat + 1, 65536);
                end

                holdingRegisters = pack_openplc_plant_registers( ...
                    measurements(1), measurements(2), measurements(3), ...
                    measurements(4), measurements(5), measurements(6), ...
                    measurements(7), measurements(8), obj.Heartbeat);

                % mixer feedback follows its command since theres no separate mixer dynamics here
                digitalFeedback = [ ...
                    feedback(1:5), ...
                    digitalCommands(6), ...
                    feedback(6:7)];

                write(obj.Client, 'holdingregs', 1, holdingRegisters);
                write(obj.Client, 'coils', 1, double(digitalFeedback));

                % reuse the bridge connection for the demo; without the flag, the hmi owns C101-C106
                obj.ExchangeCount = obj.ExchangeCount + 1;
                obj.applyActiveTestSequence();

                commHealthy = 1;
                obj.WarningIssued = 0;
                obj.clearLastCommunicationError();
            catch ME
                obj.handleCommunicationError(ME);
            end
        end

        function resetImpl(obj)
            obj.resetStates();
        end

        function releaseImpl(obj)
            obj.Client = [];
        end

        function validateInputsImpl(~, measurements, feedback)
            validateattributes(measurements, {'numeric'}, ...
                {'real', 'vector', 'numel', 8}, '', 'measurements');
            validateattributes(feedback, {'numeric', 'logical'}, ...
                {'real', 'vector', 'numel', 7}, '', 'feedback');
        end

        function sampleTime = getSampleTimeImpl(obj)
            sampleTime = createSampleTime(obj, ...
                'Type', 'Discrete', ...
                'SampleTime', obj.SampleTime_s);
        end

        function [size1, size2, size3, size4, size5] = getOutputSizeImpl(~)
            [size1, size2, size3, size4, size5] = ...
                OpenPLCModbusBridge.outputPortSizes();
        end

        function supported = supports1DVectorsImpl(~)
            supported = true;
        end

        function [type1, type2, type3, type4, type5] = getOutputDataTypeImpl(~)
            type1 = 'double';
            type2 = 'double';
            type3 = 'double';
            type4 = 'double';
            type5 = 'double';
        end

        function [fixed1, fixed2, fixed3, fixed4, fixed5] = isOutputFixedSizeImpl(~)
            fixed1 = true;
            fixed2 = true;
            fixed3 = true;
            fixed4 = true;
            fixed5 = true;
        end

        function [complex1, complex2, complex3, complex4, complex5] = isOutputComplexImpl(~)
            complex1 = false;
            complex2 = false;
            complex3 = false;
            complex4 = false;
            complex5 = false;
        end

        function [feedthrough1, feedthrough2] = ...
                isInputDirectFeedthroughImpl(~, ~, ~)
            % the plant has state, so these direct inputs dont create an algebraic loop
            feedthrough1 = true;
            feedthrough2 = true;
        end

        function [stateSize, stateDataType, stateComplexity] = ...
                getDiscreteStateSpecificationImpl(~, propertyName)
            % each DiscreteState needs a size and type for the system block
            switch char(propertyName)
                case { ...
                        'Heartbeat', ...
                        'LastAIT201', ...
                        'ExchangeCount', ...
                        'ReconnectCountdown', ...
                        'WarningIssued'}
                    stateSize = [1 1];
                    stateDataType = 'double';
                    stateComplexity = false;
                otherwise
                    error( ...
                        'WaterProject:UnknownBridgeDiscreteState', ...
                        'Unknown OpenPLCModbusBridge discrete state: %s', ...
                        char(propertyName));
            end
        end

        function icon = getIconImpl(~)
            icon = sprintf('OpenPLC Modbus\n127.0.0.1:5020');
        end

        function [name1, name2] = getInputNamesImpl(~)
            name1 = 'Measurements';
            name2 = 'Feedback';
        end

        function [name1, name2, name3, name4, name5] = getOutputNamesImpl(~)
            name1 = 'AnalogCmd';
            name2 = 'DigitalCmd';
            name3 = 'Faults';
            name4 = 'CommOK';
            name5 = 'Diagnostics';
        end
    end

    methods (Static, Access = protected)
        function simulationMode = getSimulateUsingImpl
            % this method needs to be static for the system block
            simulationMode = 'Interpreted execution';
        end
    end

    methods (Access = private)
        function resetStates(obj)
            obj.Heartbeat = 0;
            obj.LastAIT201 = 0;
            obj.ExchangeCount = 0;
            obj.ReconnectCountdown = 0;
            obj.WarningIssued = 0;
        end

        function applyActiveTestSequence(obj)
            if obj.isStage5SequenceEnabled()
                obj.applyStage5RequestSequence();
            else
                obj.applyDemoRequestSequence();
            end
        end

        function applyDemoRequestSequence(obj)
            if ~obj.isDemoSequenceEnabled()
                return;
            end

            % hold requests for three 100 ms updates so the plc catches the edge; leave a low gap between them
            requests = zeros(1, 6);
            if obj.ExchangeCount >= 11 && obj.ExchangeCount <= 13
                requests(3) = 1; % C103: reset
            elseif obj.ExchangeCount >= 17 && obj.ExchangeCount <= 19
                requests(4) = 1; % C104: automatic mode
            elseif obj.ExchangeCount >= 23 && obj.ExchangeCount <= 25
                requests(1) = 1; % C101: system start
            end
            write(obj.Client, 'coils', 101, requests);
        end

        function applyStage5RequestSequence(obj)
            % only the commissioning run enables this; normal runs leave hmi requests and fault coils alone
            requests = zeros(1, 6);
            if obj.ExchangeCount >= 11 && obj.ExchangeCount <= 13
                requests(3) = 1; % C103: reset
            elseif obj.ExchangeCount >= 17 && obj.ExchangeCount <= 19
                requests(4) = 1; % C104: automatic mode
            elseif obj.ExchangeCount >= 23 && obj.ExchangeCount <= 25
                requests(1) = 1; % C101: system start
            end
            write(obj.Client, 'coils', 101, requests);

            % HR201-HR205: pressure, concentration, fill target, mix time, manual dose
            if obj.ExchangeCount <= 30
                write(obj.Client, 'holdingregs', 201, ...
                    [4000 120 500 1 500]);
            end

            % trip the lead pump from 60 to 75 s to check takeover
            faults = zeros(1, 4);
            if obj.ExchangeCount >= 600 && obj.ExchangeCount <= 750
                faults(1) = 1;
            end
            write(obj.Client, 'coils', 251, faults);
        end

        function enabled = isDemoSequenceEnabled(~)
            enabled = false;
            try
                enabled = isappdata(0, ...
                    'WaterProjectOpenPLCDemoSequence') && ...
                    logical(getappdata(0, ...
                    'WaterProjectOpenPLCDemoSequence'));
            catch
            end
        end

        function enabled = isStage5SequenceEnabled(~)
            enabled = false;
            try
                enabled = isappdata(0, ...
                    'WaterProjectOpenPLCStage5Sequence') && ...
                    logical(getappdata(0, ...
                    'WaterProjectOpenPLCStage5Sequence'));
            catch
            end
        end

        function recordLastCommunicationError(~, ME)
            try
                setappdata(0, 'WaterProjectOpenPLCLastError', ...
                    sprintf('%s: %s', ME.identifier, ME.message));
            catch
            end
        end

        function clearLastCommunicationError(~)
            try
                if isappdata(0, 'WaterProjectOpenPLCLastError')
                    rmappdata(0, 'WaterProjectOpenPLCLastError');
                end
            catch
            end
        end

        function tryConnect(obj)
            try
                obj.Client = modbus( ...
                    'tcpip', ...
                    obj.Host, ...
                    obj.Port, ...
                    'Timeout', ...
                    obj.Timeout_s);
                obj.ReconnectCountdown = 0;
                obj.WarningIssued = 0;
            catch ME
                obj.Client = [];
                obj.ReconnectCountdown = max(1, round(1 / obj.SampleTime_s));
                obj.recordLastCommunicationError(ME);
                if obj.WarningIssued == 0
                    warning( ...
                        'WaterProject:OpenPLCConnection', ...
                        ['OpenPLC Modbus connection is unavailable. ' ...
                         'The plant commands are being held safe at zero. %s'], ...
                        ME.message);
                    obj.WarningIssued = 1;
                end
            end
        end

        function handleCommunicationError(obj, ME)
            obj.Client = [];
            obj.ReconnectCountdown = max(1, round(1 / obj.SampleTime_s));
            obj.recordLastCommunicationError(ME);
            if obj.WarningIssued == 0
                warning( ...
                    'WaterProject:OpenPLCCommunication', ...
                    ['OpenPLC Modbus exchange failed. The bridge will retry ' ...
                     'and all plant commands are safe at zero. %s'], ...
                    ME.message);
                obj.WarningIssued = 1;
            end
        end
    end
end
