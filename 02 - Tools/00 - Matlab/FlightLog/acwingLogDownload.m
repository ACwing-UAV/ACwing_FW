function files = acwingLogDownload(port, ids, outDir)
%ACWINGLOGDOWNLOAD  Lists, downloads or erases the flight logs of the FC (USB or BLE).
%
%   acwingLogDownload('COM7')                     % prints the list of logs (USB)
%   files = acwingLogDownload('COM7', 'all')      % downloads every log
%   files = acwingLogDownload('COM7', [3 4], 'logs')
%   acwingLogDownload('COM7', 'erase')            % erases every log (asks to type YES)
%   files = acwingLogDownload('ble', 'all')       % BLE: the LAST FLIGHT only
%   files = acwingLogDownload('ble:ACwFC', 'all') % BLE, board by name or address
%
%   USB: the FC enumerates as a virtual COM port (CDC, VID/PID 0483:5740); the
%   baud rate is ignored. Every log, and erase. From firmware 0.6.0 the port is
%   a ground link that also streams telemetry: it is skipped here.
%   BLE: the same commands, written raw to the CMD characteristic; the drone
%   lists and sends only the newest log, erase is USB only (HANDOVER_APP §4).
%   port may also be a struct with fields write(bytes), read() -> uint8 row and
%   raw (true = BLE framing), e.g. an emulated drone in a test.
%
%   Each log is saved as acwing_flight_<id>.bin; decode it with acwingLogDecode.
%   Downloads are refused while the drone is armed.
%   Protocol: App/app/log_task.h. Python equivalent: "acwing-tools log
%   list|get|erase [--ble]" (acwing_tools/flightlog.py, FcLink): same commands,
%   chunks and timeouts. Needs MATLAB R2019b or newer (serialport, ble).

if nargin < 2, ids = []; end
if nargin < 3, outDir = '.'; end
files = {};
[L, cleanup] = openLink(port); %#ok<ASGLU>   cleanup closes the link on return or error
st.seq = 0; st.rest = uint8([]);

% SET_TIME (0x43): the wall clock for the next logs (LR_TIME); older firmware refuses it
[t, tz] = nowUtc();
try
    st = command(L, st, 67, [typecast(uint32(floor(t)), 'uint8'), ...
                             typecast(uint16(floor(mod(t, 1) * 1000)), 'uint8'), typecast(int16(tz), 'uint8')]);
catch
end

if ischar(ids) && strcmpi(ids, 'erase')
    if L.raw, error('acwingLogDownload:erase', 'erase is USB only: give the FC USB port'); end
    if ~(isfield(L, 'yes') && L.yes)
        a = input('Erase EVERY log on the drone? Type YES: ', 's');
        if ~strcmp(a, 'YES'), fprintf('Not erased.\n'); return; end
    end
    st = command(L, st, 82, uint8('WIPE')); %#ok<NASGU>  LOG_ERASE (0x52): IN_PROGRESS, then OK
    fprintf('Every log erased.\n');
    return;
end

[st, fr] = command(L, st, 80, uint8([]));               % LOG_LIST (0x50)
E = struct('id', {}, 'flags', {}, 'bytes', {}, 't0', {});
for f = fr
    if f.type == 16 && numel(f.payload) >= 15
        p = f.payload;
        E(end+1) = struct('id', double(typecast(p(1:2), 'uint16')), 'flags', p(5), ...
                          'bytes', double(typecast(p(12:15), 'uint32')), ...
                          't0', double(typecast(p(8:11), 'uint32')) / 1000); %#ok<AGROW>
    end
end
if isempty(E), fprintf('No logs on the drone.\n'); return; end
if isempty(ids)
    for k = 1:numel(E)
        fl = ''; if bitand(E(k).flags, 1), fl = 'truncated'; end
        if bitand(E(k).flags, 2), fl = strtrim([fl ' recording']); end
        fprintf('  log %5d  %8.0f kB  started %8.1f s after power-on  %s\n', E(k).id, E(k).bytes/1024, E(k).t0, fl);
    end
    if L.raw, fprintf('  (Bluetooth: the last flight only; every log over USB)\n'); end
    return;
end
if ischar(ids) && strcmpi(ids, 'all'), ids = [E.id]; end
if ~exist(outDir, 'dir'), mkdir(outDir); end
CH = 256 * 1024;                                       % per LOG_READ: a lost frame costs one chunk
if L.raw, CH = 32 * 1024; end
for k = 1:numel(E)
    if ~any(ids == E(k).id), continue; end
    data = zeros(1, E(k).bytes, 'uint8') + 255;
    for off = 0:CH:E(k).bytes - 1
        n = min(CH, E(k).bytes - off);
        args = [typecast(uint16(E(k).id), 'uint8'), typecast(uint32(off), 'uint8'), typecast(uint32(n), 'uint8')];
        [st, fr] = command(L, st, 81, args);           % LOG_READ (0x51): LOG_DATA ..., then ACK
        got = 0;
        for f = fr
            if f.type == 17 && numel(f.payload) > 6 && double(typecast(f.payload(1:2), 'uint16')) == E(k).id
                o = double(typecast(f.payload(3:6), 'uint32'));
                d = f.payload(7:end);
                if o >= off && o + numel(d) <= E(k).bytes
                    data(o + (1:numel(d))) = d;
                    got = got + numel(d);
                end
            end
        end
        if got ~= n, error('acwingLogDownload:short', 'log %d: %d of %d bytes at offset %d', E(k).id, got, n, off); end
        fprintf('\r  log %d: %3d %%', E(k).id, round(100 * (off + n) / E(k).bytes));
    end
    fprintf('\n');
    files{end+1} = fullfile(outDir, sprintf('acwing_flight_%05d.bin', E(k).id)); %#ok<AGROW>
    fid = fopen(files{end}, 'w'); fwrite(fid, data, 'uint8'); fclose(fid);
    fprintf('  -> %s\n', files{end});
end
end

% ------------------------------------------------------------------------
function [L, cleanup] = openLink(port)
% L.write(bytes), L.read() -> uint8 row, L.raw (BLE: commands not wrapped in a CMD frame)
if isstruct(port)
    L = port;
    if ~isfield(L, 'raw'), L.raw = false; end
    if ~isfield(L, 'timeout'), L.timeout = 2; end
    cleanup = [];
    return;
end
port = char(port);
if strncmpi(port, 'ble', 3)
    SVC = "7E4A0001-2C6B-4F8E-9A5D-3B1C0E1D2F30"; TLM = "7E4A0002-2C6B-4F8E-9A5D-3B1C0E1D2F30";
    CMD = "7E4A0003-2C6B-4F8E-9A5D-3B1C0E1D2F30";                         % = ble_tlm_svc.c
    target = strtrim(port(4:end)); if ~isempty(target) && target(1) == ':', target = target(2:end); end
    if isempty(target), target = findBoard(); end
    dev = ble(target);
    buf = containers.Map({'d'}, {uint8([])});                 % handle: filled by the callback
    tlm = characteristic(dev, SVC, TLM);
    tlm.DataAvailableFcn = @(src, ~) bleAppend(src, buf);
    subscribe(tlm);
    cmd = characteristic(dev, SVC, CMD);
    write(cmd, uint8([2, 240]), "withoutresponse");           % STATE at 1 Hz: the radio time goes to the log
    L = struct('raw', true, 'timeout', 5, 'write', @(b) write(cmd, b, "withoutresponse"), 'read', @() bleTake(buf));
    cleanup = onCleanup(@() bleClose(cmd, tlm, dev));
else
    sp = serialport(port, 115200, 'Timeout', 2);
    try, setDTR(sp, true); catch, end                         % the FC opens its USB link on DTR (or on commands)
    flush(sp);
    L = struct('raw', false, 'timeout', 2, 'write', @(b) write(sp, b, 'uint8'), 'read', @() serialTake(sp));
    cleanup = onCleanup(@() delete(sp));
end
end

function b = serialTake(sp)
n = sp.NumBytesAvailable;
if n == 0, b = uint8([]); else, b = uint8(read(sp, n, 'uint8')); end
end

function bleAppend(src, buf)
pkt = uint8(read(src, 'oldest'));
buf('d') = [buf('d'), pkt(:).'];
end

function b = bleTake(buf)
b = buf('d'); buf('d') = uint8([]);
end

function bleClose(cmd, tlm, dev)
try, write(cmd, uint8([2, 4]), "withoutresponse"); catch, end   % STATE back to the default 60 Hz
try, unsubscribe(tlm); catch, end
clear dev
end

function target = findBoard()
T = blelist("Timeout", 5);
names = string(T.Name);
k = find(startsWith(names, "ACw") | startsWith(names, "IEKF") | names == "STM32WB", 1);
if isempty(k), error('acwingLogDownload:ble', 'no ACwing board advertising: give its name or address'); end
target = string(T.Address(k));
end

function [t, tz] = nowUtc()
try
    t = posixtime(datetime('now', 'TimeZone', 'UTC'));
    tz = round(minutes(tzoffset(datetime('now', 'TimeZone', 'local'))));
catch                                                     % Octave (tests): no datetime
    t = time(); tz = 0;
end
end

function [st, frames] = command(L, st, op, args)
% Sends a command and collects its LOG_ENTRY / LOG_DATA answers until its ACK.
% Only those frames extend the wait: the telemetry that a ground link (firmware
% >= 0.6.0) streams meanwhile is dropped and never keeps a lost ACK waiting.
st.seq = mod(st.seq + 1, 256);
c = uint8([op, st.seq, args]);
if L.raw
    L.write(c);
else
    body = uint8([32, numel(c), c]);                        % CMD frame (0x20)
    crc = crc16ccitt(body);
    L.write([uint8([165 90]), body, uint8(bitand(crc, 255)), uint8(bitshift(crc, -8))]);
end
frames = struct('type', {}, 'payload', {});
tLast = tic; limit = L.timeout;
while toc(tLast) < limit
    b = L.read();
    if isempty(b), pause(0.002); continue; end
    [fr, st.rest] = parseTelemetry([st.rest, b(:).']);
    for f = fr
        if f.type == 14 && numel(f.payload) >= 4 && f.payload(1) == op && f.payload(2) == st.seq
            res = f.payload(3);
            if res == 5, limit = max(limit, 300); tLast = tic; continue; end      % IN_PROGRESS (erase)
            if res ~= 0, error('acwingLogDownload:ack', '%s', refusal(op, res, f.payload(4))); end
            return;
        end
        if f.type == 16 || f.type == 17                         % LOG_ENTRY, LOG_DATA
            frames(end+1) = f; %#ok<AGROW>
            tLast = tic;
        end
    end
end
error('acwingLogDownload:timeout', 'no answer to 0x%02X: is this the FC USB port (or the board over BLE)?', op);
end

function s = refusal(op, res, why)
R = {'OK', 'REJECTED', 'BUSY', 'INVALID', 'UNSUPPORTED', 'IN_PROGRESS'};
W = containers.Map('KeyType', 'double', 'ValueType', 'any');
W(8) = 'ARMED: disarm to download'; W(14) = 'OUT_OF_RANGE (unknown log, or not the last flight over BLE)';
W(16) = 'SENSOR_MISSING (no log flash on this board)';
r = sprintf('%d', res); if res + 1 <= numel(R), r = R{res + 1}; end
w = ''; if isKey(W, double(why)), w = [' ' W(double(why))]; elseif why, w = sprintf(' reason %d', why); end
s = sprintf('command 0x%02X refused: %s%s', op, r, w);
end
