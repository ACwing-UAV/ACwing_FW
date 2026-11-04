function testFlightLog()
%TESTFLIGHTLOG  Tests of the MATLAB flight-log tools against emulated drones
%   (MATLAB or Octave): acwingLogDecode on the firmware test vector, the same
%   numbers as the Python decoder; acwingLogDownload over USB (every log, erase),
%   over BLE (last flight only) and with a firmware >= 0.6.0 that streams
%   telemetry on the USB port and loses an ACK (must time out, not hang).
%   Python twins: tests/test_flightlog.py (FakeFC, FakeBle, ChattyFC).
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'FlightLog'), fullfile(here, '..', 'Telemetry'));
sample = fullfile(here, '..', '..', '01 - Python', 'tests', 'data', 'flightlog_sample.bin');
fid = fopen(sample, 'r'); bin = fread(fid, inf, 'uint8=>uint8').'; fclose(fid);
n = 0;

% ---- decoder: the firmware test vector (HANDOVER_APP §6)
F = acwingLogDecode(sample);
n = n + check(F.sectors == 6 && F.bad == 0 && F.unknown == 0, 'sectors 6, 0 bad, 0 unknown');
n = n + check(strcmp(F.info.fw, '0.5.0') && F.info.hw_rev == 'N', 'INFO fw 0.5.0, hwRev N');
n = n + check(F.att.n == 350 && F.vert.n == 350 && all(abs(diff(F.att.t) - 0.02) < 1e-9), '350 ATT and VERT at 20 ms');
n = n + check(F.flight.n == 70 && F.power.n == 70 && abs(F.power.vbat(1) - 3.9) < 1e-9 && F.power.cells(1) == 1 && F.power.present(1) == 1, ...
              '70 FLIGHT and 70 POWER, 3.9 V, 1S, present');
n = n + check(isequal(F.event.name, {'ARMED', 'DISARMED'}), 'events ARMED, DISARMED');
n = n + check(F.time.n == 1 && ~isempty(F.wallclock), 'LR_TIME and wall clock');
bad = bin; bad(4096 + 30) = bitxor(bad(4096 + 30), 255);              % a byte of a record in sector 2
G = acwingLogDecode(bad);
n = n + check(G.sectors == 6 && G.bad == 1 && G.att.n < 350, 'damaged record: rest of that sector skipped only');
bad = bin; bad(4096 + 5) = bitxor(bad(4096 + 5), 255);                % sector 2 header
G = acwingLogDecode(bad);
n = n + check(G.sectors == 5 && G.bad == 1, 'damaged header: that sector skipped only');

% ---- USB: list, read in chunks, erase
other = uint8(repmat(0:255, 1, 32));
D = fakeDrone(false, {3, bin; 7, other}, false);
out = tempname(); mkdir(out);
files = acwingLogDownload(D.link, 'all', out);
n = n + check(numel(files) == 2 && isequal(readBin(files{1}), bin) && isequal(readBin(files{2}), other), 'USB: both logs, byte-exact');
n = n + check(D.state('time') > 1.7e9, 'SET_TIME sent before listing');
L = D.link; L.yes = true;
acwingLogDownload(L, 'erase');
n = n + check(isempty(keys(D.state('logs'))), 'USB: erase (IN_PROGRESS then OK)');
D = fakeDrone(false, {3, bin}, true);                                % armed
n = n + check(throws(@() acwingLogDownload(D.link, 'all', out), 'ARMED'), 'READ while armed: "disarm to download"');

% ---- BLE: last flight only, raw commands, 32 KB chunks
D = fakeDrone(true, {3, bin; 7, other}, false);
files = acwingLogDownload(D.link, 'all', out);
n = n + check(numel(files) == 1 && ~isempty(strfind(files{1}, '00007')) && isequal(readBin(files{1}), other), 'BLE: the newest log only');
n = n + check(throws(@() acwingLogDownload(D.link, 'erase'), 'USB only'), 'BLE: erase refused locally');

% ---- firmware >= 0.6.0: telemetry on the USB port, lost ACK -> timeout, not a hang
D = fakeDrone(false, {3, bin}, false); D.state('chatty') = true; D.state('dropList') = true;
L = D.link; L.timeout = 0.3;
t0 = tic;
n = n + check(throws(@() acwingLogDownload(L), 'no answer'), 'lost ACK with telemetry streaming: timeout');
n = n + check(toc(t0) < 5, 'timeout within the limit');
D = fakeDrone(false, {3, bin}, false); D.state('chatty') = true;
files = acwingLogDownload(D.link, 'all', out);
n = n + check(isequal(readBin(files{1}), bin), 'telemetry interleaved with LOG_DATA: still byte-exact');
fprintf('testFlightLog: %d passed\n', n);
end

% ------------------------------------------------------------------------
function ok = check(c, msg)
if ~c, error('testFlightLog:fail', 'FAIL: %s', msg); end
ok = 1;
end

function t = throws(fn, txt)
t = false;
try, fn(); catch e, t = ~isempty(strfind(e.message, txt)); if ~t, disp(e.message); end, end
end

function d = readBin(f)
fid = fopen(f, 'r'); d = fread(fid, inf, 'uint8=>uint8').'; fclose(fid);
end

function D = fakeDrone(isBle, logs, armed)
% the drone as log_task.c answers it: entries/data before the ACK, <= 240-byte LOG_DATA
S = containers.Map();
lg = containers.Map('KeyType', 'double', 'ValueType', 'any');
for k = 1:size(logs, 1), lg(logs{k, 1}) = logs{k, 2}; end
S('logs') = lg; S('out') = uint8([]); S('rx') = uint8([]); S('armed') = armed; S('ble') = isBle;
S('time') = 0; S('chatty') = false; S('dropList') = false;
D.state = S;
D.link = struct('raw', isBle, 'timeout', 2, 'write', @(b) fakeWrite(S, b), 'read', @() fakeRead(S));
end

function fakeWrite(S, b)
b = uint8(b(:).');
if S('ble'), fakeCmd(S, b(1), b(2), b(3:end)); return; end
[fr, S('rx')] = parseTelemetry([S('rx'), b]);
for f = fr
    if f.type == 32, fakeCmd(S, f.payload(1), f.payload(2), f.payload(3:end)); end
end
end

function b = fakeRead(S)
o = S('out');
if S('chatty'), o = [o, frame(6, zeros(1, 25, 'uint8'))]; end       % FLIGHT, as the ground link
k = min(numel(o), 1000);                                               % the USB delivers in pieces
b = o(1:k); S('out') = o(k+1:end);
end

function emit(S, f), S('out') = [S('out'), f]; end
function ack(S, op, seq, res, why), emit(S, frame(14, uint8([op, seq, res, why]))); end

function f = frame(type, p)
body = uint8([type, numel(p), p]);
c = crc16ccitt(body);
f = [uint8([165 90]), body, uint8(bitand(c, 255)), uint8(bitshift(c, -8))];
end

function fakeCmd(S, op, seq, a)
lg = S('logs'); ids = sort(cell2mat(keys(lg)));
if S('ble') && ~isempty(ids), ids = ids(end); end                     % BLE: the last flight only
switch double(op)
    case 67                                                            % SET_TIME
        S('time') = double(typecast(uint8(a(1:4)), 'uint32')); ack(S, op, seq, 0, 0);
    case 80                                                            % LOG_LIST
        for k = 1:numel(ids)
            nb = numel(lg(ids(k)));
            emit(S, frame(16, [typecast(uint16(ids(k)), 'uint8'), uint8([k-1, numel(ids), 0]), ...
                               typecast(uint16(nb / 4096), 'uint8'), typecast(uint32(1000 * ids(k)), 'uint8'), typecast(uint32(nb), 'uint8')]));
        end
        if ~S('dropList'), ack(S, op, seq, 0, 0); end
    case 81                                                            % LOG_READ
        if S('armed'), ack(S, op, seq, 1, 8); return; end
        id = double(typecast(uint8(a(1:2)), 'uint16')); off = double(typecast(uint8(a(3:6)), 'uint32'));
        len = double(typecast(uint8(a(7:10)), 'uint32'));
        if ~any(ids == id), ack(S, op, seq, 1 + 2 * ~S('ble'), 14); return; end
        d = lg(id); d = d(off+1:min(numel(d), off + len));
        for k = 0:240:numel(d) - 1
            emit(S, frame(17, [typecast(uint16(id), 'uint8'), typecast(uint32(off + k), 'uint8'), d(k+1:min(numel(d), k + 240))]));
        end
        ack(S, op, seq, 0, 0);
    case 82                                                            % LOG_ERASE
        if S('ble'), ack(S, op, seq, 4, 0); return; end
        ack(S, op, seq, 5, 0); S('logs') = containers.Map('KeyType', 'double', 'ValueType', 'any'); ack(S, op, seq, 0, 0);
    otherwise
        ack(S, op, seq, 4, 0);
end
end
