function L = iekfLiveViewer(transport, target, baud, logDir)
%IEKFLIVEVIEWER  3D triad following, in real time, the attitude estimated by the firmware.
%
%   iekfLiveViewer("serial", "COM6")                 % USB (ST-LINK VCP), 921600 baud
%   iekfLiveViewer("serial", "/dev/ttyACM0", 460800)
%   iekfLiveViewer("ble")                            % BLE, board found automatically, 60 Hz
%   iekfLiveViewer("ble", "IEKF")               % BLE, by name (or address), 60 Hz
%   iekfLiveViewer("ble", "IEKF", 8)            % BLE, STATE every 8 -> 30 Hz
%
%   Both links carry the SAME frames (STATE, INIT, STATUS): the log, the .bin
%   and everything downstream are identical. Over BLE the STATE rate is lower
%   (3rd argument = decimation, default 4) and REPLAY is never sent.
%   Automatic discovery picks the first board advertising a name that starts
%   with "IEKF" or equals "STM32WB" (the CubeMX default). blelist lists them all.
%
%   World frame NWU (x = magnetic north, y = west, z = up), defined by the
%   firmware through the static calibration: keep the board STILL ~2 s after reset.
%   View: x to the right, y into the screen, z up.
%   Start the viewer BEFORE resetting the board if you want the complete log.
%   Close the figure to stop (with Ctrl+C the last autosave remains, at most
%   10 s old). With APP_TLM_REPLAY = 1 use tlmCapture.m instead: the viewer
%   cannot keep up with ~44 kB/s.
%
%   LOG: the full file path is printed at start-up. The file is saved every 10 s
%   (autosave) and on close; on close the log is also returned as output and
%   copied into the base workspace as variable iekfLog:
%       L = iekfLiveViewer("serial","COM6");
%   logDir (optional) selects the folder, default the current folder.
%
%   serialport: R2019b+.  ble: R2019b+, Windows 10+ or macOS, Bluetooth 4.0+ adapter.

if nargin < 1, transport = "serial"; end
isBle = strcmpi(string(transport), "ble");
if nargin < 2, if isBle, target = ""; else, target = "COM6"; end, end
if nargin < 3 || isempty(baud), if isBle, baud = 4; else, baud = 921600; end, end
if nargin < 4 || isempty(logDir), logDir = pwd; end

SVC_UUID = "7E4A0001-2C6B-4F8E-9A5D-3B1C0E1D2F30";   % = ble_tlm_svc.c
CHR_UUID = "7E4A0002-2C6B-4F8E-9A5D-3B1C0E1D2F30";   % TLM: notify, byte stream
CMD_UUID = "7E4A0003-2C6B-4F8E-9A5D-3B1C0E1D2F30";   % CMD: write without response

% ------------------------------------------------------------------ state
dev = []; bleDev = []; bleChr = []; bleRx = uint8([]); nBleNotif = 0;
L = newLog(6000); nLog = 0;
lastSeq = NaN; seqOffset = 0; nLost = 0; nFrames = 0; nStatus = 0;
seqStep = Inf;                 % STATE decimation seen in the stream (1, 4, 8, ...)
buf = uint8([]);
lastUptime = -1; nResets = 0; finished = false;
logFile = fullfile(logDir, sprintf('iekf_log_%s.mat', datestr(now, 'yyyymmdd_HHMMSS'))); %#ok<TNOW1,DATST>
tSave = tic;
binFile = strrep(logFile, '.mat', '.bin');           % raw capture (for compareEmbeddedVsMatlab)
binFid  = fopen(binFile, 'w');
nReplay = 0;
fprintf('Log: %s\nRaw capture: %s\n', logFile, binFile);
latestQ = [1 0 0 0]; latestS = []; latestStatus = []; tStart = tic;
lagRef = [];                                       % [t_pc t_fw] of the first STATE

[fig, hT, hTxt] = buildFigure();

% Ctrl+C cannot be intercepted: in that case the last autosave remains (<= 10 s).
% Any other error goes through the catch, which saves before rethrowing.
try
    switch lower(string(transport))
        case "serial"
            dev = serialport(target, baud, "Timeout", 1);
            flush(dev);
            while ishghandle(fig)
                nb = dev.NumBytesAvailable;
                if nb > 0
                    consumeBytes(uint8(read(dev, nb, "uint8")));
                else
                    pause(0.005);
                end
            end

        case "ble"
            if strlength(string(target)) == 0, target = findIekfBoard(); end
            fprintf('Connecting to "%s" over BLE...\n', target);
            bleDev = ble(target);
            bleChr = characteristic(bleDev, SVC_UUID, CHR_UUID);
            bleChr.DataAvailableFcn = @onBle;
            subscribe(bleChr);                         % the board starts streaming + sends INIT
            try                                        % optional: STATE decimation
                cmdChr = characteristic(bleDev, SVC_UUID, CMD_UUID);
                write(cmdChr, uint8([2, max(1, min(240, round(baud)))]), "withoutresponse");
            catch err
                fprintf(2, 'CMD not written (%s): default decimation in use\n', err.message);
            end
            fprintf('Connected: STATE every %d acc updates (%.0f Hz)\n', round(baud), 240/round(baud));
            while ishghandle(fig)
                if ~isempty(bleRx)
                    chunk = bleRx; bleRx = uint8([]);  % notifications arrived meanwhile
                    consumeBytes(chunk);
                else
                    pause(0.01);                       % callbacks run here
                end
            end

        otherwise
            error('transport must be "serial" or "ble"');
    end
catch err
    finish();
    rethrow(err);
end
finish();          % normal close: save and prepare the output

% ================================================================== nested
    function consumeBytes(chunk)
        % same path for USB and BLE: raw .bin, parser, dispatch
        if binFid > 0, fwrite(binFid, chunk, 'uint8'); end
        buf = [buf, chunk];
        [frames, buf] = parseTelemetry(buf, 4);   % REPLAY: only in the .bin
        for k = 1:numel(frames)
            switch frames(k).type
                case 1, handleState(decodeState(frames(k).payload));
                case 2, L.init = decodeInit(frames(k).payload);
                case 3, handleStatus(decodeStatus(frames(k).payload));
                case 4, nReplay = nReplay + 1;     % only in the .bin capture
            end
        end
        refresh();
        if toc(tSave) > 10, saveLog(); tSave = tic; end
    end

    function handleState(s)
        if ~isnan(lastSeq)
            d = mod(s.seq - lastSeq, 65536);
            if d == 0, return; end                 % duplicate
            seqStep = min(seqStep, d);             % decimation of the stream
            nLost = nLost + max(0, round(d / seqStep) - 1);
            if s.seq < lastSeq, seqOffset = seqOffset + 65536; end   % wrap-around
        end
        lastSeq = s.seq;
        nFrames = nFrames + 1;

        if nLog == numel(L.seq), L = growLog(L); end
        nLog = nLog + 1;
        L.seq(nLog) = s.seq + seqOffset;  L.t_us(nLog) = s.t_us;  L.q(:,nLog) = s.q(:);
        L.bias(:,nLog) = s.bias(:); L.acc(:,nLog) = s.acc(:);
        L.gyro(:,nLog) = s.gyro(:); L.mag(:,nLog) = s.mag(:); L.flags(nLog) = s.flags;
        L.t_pc(nLog) = toc(tStart);
        if isempty(lagRef) || s.t_us*1e-6 < lagRef(2), lagRef = [L.t_pc(nLog), s.t_us*1e-6]; end
        latestQ = s.q; latestS = s;
    end

    function handleStatus(st)
        latestStatus = st;
        nStatus = nStatus + 1;
        if lastUptime >= 0 && st.uptime_s < lastUptime   % the board rebooted
            nResets = nResets + 1;
            L.resetAt(end+1) = nLog + 1;
            fprintf(2, 'WARNING: board reboot detected (sample %d)\n', nLog + 1);
            lastSeq = NaN;
        end
        lastUptime = st.uptime_s;
        if isempty(L.status), L.status = st; else, L.status(end+1) = st; end
    end

    function onBle(src, ~)
        % one notification = up to ATT_MTU-3 bytes of the frame stream (frames may
        % be split across notifications): only append, parsing is in the main loop
        pkt = uint8(read(src, 'oldest')); bleRx = [bleRx, pkt(:).'];
        nBleNotif = nBleNotif + 1;
    end

    function refresh()
        if ~ishghandle(fig), return; end
        R = quat2rotmLocal(latestQ);
        hT.Matrix = [R, zeros(3,1); 0 0 0 1];
        eul = rad2deg([atan2(R(2,1), R(1,1)); asin(max(-1,min(1,-R(3,1)))); atan2(R(3,2), R(3,3))]);
        rate = nFrames / max(toc(tStart), 1e-3);
        txt = sprintf(['roll %7.1f   pitch %7.1f   yaw %7.1f  [deg]\n' ...
                       'packets %d  (%.0f Hz)   lost %d   log %d samples'], ...
                       eul(3), eul(2), eul(1), nFrames, rate, nLost, nLog);
        if ~isempty(latestS)
            % display lag: elapsed PC time - elapsed firmware time
            lag = (toc(tStart) - lagRef(1)) - (latestS.t_us*1e-6 - lagRef(2));
            txt = [txt, sprintf('   lag %.2f s', lag)];
            if lag > 0.5
                txt = [txt, '  <-- VIEWER CANNOT KEEP UP'];
            end
            txt = [txt, sprintf('\nbias [deg/s]  %6.3f %6.3f %6.3f', rad2deg(latestS.bias))];
            fl = {};
            if bitand(latestS.flags, 4),  fl{end+1} = 'ACC de-weighted'; end %#ok<AGROW>
            if bitand(latestS.flags, 8),  fl{end+1} = 'MAG de-weighted'; end %#ok<AGROW>
            if bitand(latestS.flags, 16), fl{end+1} = 'OVERRUN';       end %#ok<AGROW>
            if bitand(latestS.flags, 32), fl{end+1} = 'STILL';        end %#ok<AGROW>
            if ~isempty(fl), txt = [txt, newline, strjoin(fl, '  |  ')]; end
        end
        if ~isempty(L.init)
            txt = [txt, sprintf('\ncalib: sigmaAcc %.4f  sigmaMag %.4f  dip %.1f deg', ...
                   L.init.sigmaAcc, L.init.sigmaMag, -asind(L.init.mRef(3)))];
        end
        if ~isempty(latestStatus) && latestStatus.ble_state > 0
            bs = {'off', 'advertising', 'connected', 'streaming'};
            txt = [txt, sprintf('\nBLE: %s  MTU %d  drop %d', bs{latestStatus.ble_state + 1}, ...
                   latestStatus.ble_mtu, latestStatus.ble_drop)];
            if latestStatus.ble_err, txt = [txt, sprintf('  err 0x%02X', latestStatus.ble_err)]; end
            if isBle, txt = [txt, sprintf('  (%d notifications)', nBleNotif)]; end
        end
        if isempty(latestS) && ~isempty(latestStatus)
            txt = sprintf('STATUS (no attitude data yet)\n%s', statusDiagnosis(latestStatus));
        end
        hTxt.String = txt;
        drawnow limitrate;
    end

    function saveLog()
        if nLog == 0 && nStatus == 0, return; end
        try
            S.L = trimLog(L, nLog);
            save(logFile, '-struct', 'S');
        catch err
            fprintf(2, 'ERROR while saving %s:\n  %s\n', logFile, err.message);
            fprintf(2, 'Use a writable folder: iekfLiveViewer("serial","COMx",921600,"C:\\path")\n');
        end
    end

    function finish()
        if finished, return; end
        finished = true;
        try
            if ~isempty(bleChr), unsubscribe(bleChr); end
        catch
        end
        dev = []; bleChr = []; bleDev = []; %#ok<SETNU>
        if ~isempty(bleRx)                  % notifications received while closing
            if binFid > 0, fwrite(binFid, bleRx, 'uint8'); end
            bleRx = uint8([]);
        end
        if binFid > 0, fclose(binFid); binFid = -1; end
        saveLog();
        L = trimLog(L, nLog);
        assignin('base', 'iekfLog', L);
        fprintf('Log: %s\n  %d STATE samples, %d STATUS, %d lost, %d reboots.\n', ...
                logFile, nLog, nStatus, nLost, nResets);
        fprintf('  Also in the workspace as variable iekfLog.\n');
        if nReplay > 0
            fprintf('  %d REPLAY packets: compareEmbeddedVsMatlab(''%s'')\n', nReplay, binFile);
        end
    end
end

% ================================================================== local functions
function target = findIekfBoard()
%FINDIEKFBOARD  Address of the first board advertising "IEKF..." or "STM32WB".
fprintf('Scanning for the board (5 s)...\n');
T = blelist("Timeout", 5);
names = string(T.Name);
k = find(startsWith(names, "IEKF") | names == "STM32WB", 1);
if isempty(k)
    disp(T(:, {'Name', 'Address', 'RSSI'}));
    error(['No IEKF board found: is it advertising (USB viewer: "BLE: advertising")? ' ...
           'Is it connected to a phone? Pass the name or the address explicitly.']);
end
target = string(T.Address(k));
fprintf('Found "%s" (%s, RSSI %d dBm)\n', names(k), target, T.RSSI(k));
end

function L = newLog(cap)
L = struct('seq', zeros(1,cap), 't_us', zeros(1,cap), 'q', zeros(4,cap), ...
           'bias', zeros(3,cap), 'acc', zeros(3,cap), 'gyro', zeros(3,cap), ...
           'mag', zeros(3,cap), 'flags', zeros(1,cap), 't_pc', zeros(1,cap));
L.init    = [];
L.status  = [];              % STATUS packets (10 Hz), struct array
L.resetAt = [];              % sample indices after a board reboot
end

function L = growLog(L)
fn = {'seq','t_us','q','bias','acc','gyro','mag','flags','t_pc'};
for i = 1:numel(fn)
    L.(fn{i}) = [L.(fn{i}), zeros(size(L.(fn{i})))];
end
end

function L = trimLog(L, n)
fn = {'seq','t_us','q','bias','acc','gyro','mag','flags','t_pc'};
for i = 1:numel(fn)
    L.(fn{i}) = L.(fn{i})(:, 1:n);
end
end

function [fig, hT, hTxt] = buildFigure()
fig = figure('Name', 'IEKF live - STM32', 'Color', 'w', 'NumberTitle', 'off');
ax  = axes(fig, 'Position', [0.05 0.22 0.9 0.75]);
hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
lim = 1.6; axis(ax, [-lim lim -lim lim -lim lim]);
xlabel(ax, 'x  (N)'); ylabel(ax, 'y  (W)'); zlabel(ax, 'z  (Up)');
view(ax, 0, 20);   % x to the right, y into the screen, z up (slight elevation)

% world frame, grey
plot3(ax, [0 1.4], [0 0], [0 0], ':', 'Color', [.5 .5 .5], 'LineWidth', 1);
plot3(ax, [0 0], [0 1.4], [0 0], ':', 'Color', [.5 .5 .5], 'LineWidth', 1);
plot3(ax, [0 0], [0 0], [0 1.4], ':', 'Color', [.5 .5 .5], 'LineWidth', 1);
text(ax, 1.5, 0, 0, 'N', 'Color', [.5 .5 .5]);
text(ax, 0, 1.5, 0, 'W', 'Color', [.5 .5 .5]);
text(ax, 0, 0, 1.5, 'Up', 'Color', [.5 .5 .5]);

% body: board + triad, inside an hgtransform
hT = hgtransform('Parent', ax);
a = 0.9; b = 0.55; c = 0.05;
V = [-a -b -c; a -b -c; a b -c; -a b -c; -a -b c; a -b c; a b c; -a b c];
F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
patch('Parent', hT, 'Vertices', V, 'Faces', F, 'FaceColor', [0.2 0.45 0.3], ...
      'FaceAlpha', 0.35, 'EdgeColor', [0.1 0.25 0.15]);
cols = [0.85 0.1 0.1; 0.1 0.6 0.1; 0.1 0.2 0.85];
lbl  = {'x_b', 'y_b', 'z_b'};
for i = 1:3
    e = zeros(1,3); e(i) = 1.25;
    line('Parent', hT, 'XData', [0 e(1)], 'YData', [0 e(2)], 'ZData', [0 e(3)], ...
         'Color', cols(i,:), 'LineWidth', 3);
    text('Parent', hT, 'Position', 1.1*e, 'String', lbl{i}, 'Color', cols(i,:), 'FontWeight', 'bold');
end

hTxt = uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
                 'Position', [0.02 0.01 0.96 0.19], 'HorizontalAlignment', 'left', ...
                 'FontName', 'FixedWidth', 'FontSize', 10, 'BackgroundColor', 'w', ...
                 'String', 'waiting for data... (keep the board still after reset)');
end
