function files = acwingLogDownload(port, ids, outDir)
%ACWINGLOGDOWNLOAD  Lists and downloads the flight logs over the FC USB port.
%
%   acwingLogDownload('COM7')                 % prints the list of logs
%   files = acwingLogDownload('COM7', 'all')  % downloads every log
%   files = acwingLogDownload('COM7', [3 4], 'logs')
%
%   The FC enumerates as a USB virtual COM port (CDC); the baud rate is
%   ignored. Each log is saved as acwing_flight_<id>.bin; decode it with
%   acwingLogDecode. Downloads are refused while the drone is armed.
%   Protocol: App/app/log_task.h. Python equivalent: "acwing-tools log get".
%   Needs MATLAB R2019b or newer (serialport).

if nargin < 2, ids = []; end
if nargin < 3, outDir = '.'; end
files = {};
sp = serialport(port, 115200, 'Timeout', 2);
cleanup = onCleanup(@() delete(sp));
flush(sp);
st.seq = 0; st.rest = uint8([]);

[st, frames] = command(sp, st, 80, uint8([]));          % LOG_LIST (0x50)
L = [];
for f = frames
    if f.type == 16 && numel(f.payload) >= 15
        p = f.payload;
        L(end+1).id = double(typecast(p(1:2), 'uint16')); %#ok<AGROW>
        L(end).flags = p(5);
        L(end).bytes = double(typecast(p(12:15), 'uint32'));
        L(end).t0 = double(typecast(p(8:11), 'uint32')) / 1000;
    end
end
if isempty(L), fprintf('No logs on the drone.\n'); return; end
if isempty(ids)
    for k = 1:numel(L)
        fl = ''; if bitand(L(k).flags, 1), fl = 'truncated'; end
        fprintf('  log %5d  %8.0f kB  started %8.1f s after power-on  %s\n', L(k).id, L(k).bytes/1024, L(k).t0, fl);
    end
    return;
end
if ischar(ids) && strcmpi(ids, 'all'), ids = [L.id]; end
if ~exist(outDir, 'dir'), mkdir(outDir); end
for k = 1:numel(L)
    if ~any(ids == L(k).id), continue; end
    data = zeros(1, L(k).bytes, 'uint8') + 255;
    CH = 256 * 1024;
    for off = 0:CH:L(k).bytes - 1
        n = min(CH, L(k).bytes - off);
        args = [typecast(uint16(L(k).id), 'uint8'), typecast(uint32(off), 'uint8'), typecast(uint32(n), 'uint8')];
        [st, frames] = command(sp, st, 81, args);       % LOG_READ (0x51)
        got = 0;
        for f = frames
            if f.type == 17 && numel(f.payload) > 6
                o = double(typecast(f.payload(3:6), 'uint32'));
                d = f.payload(7:end);
                data(o + (1:numel(d))) = d;
                got = got + numel(d);
            end
        end
        if got ~= n, error('acwingLogDownload:short', 'log %d: %d of %d bytes at %d', L(k).id, got, n, off); end
        fprintf('\r  log %d: %3d %%', L(k).id, round(100 * (off + n) / L(k).bytes));
    end
    fprintf('\n');
    files{end+1} = fullfile(outDir, sprintf('acwing_flight_%05d.bin', L(k).id)); %#ok<AGROW>
    fid = fopen(files{end}, 'w'); fwrite(fid, data, 'uint8'); fclose(fid);
    fprintf('  -> %s\n', files{end});
end
end

% ------------------------------------------------------------------------
function [st, frames] = command(sp, st, op, args)
% sends a CMD frame (type 0x20) and collects every frame until its ACK
st.seq = mod(st.seq + 1, 256);
body = uint8([32, numel(args) + 2, op, st.seq, args]);
crc = crc16ccitt(body);
write(sp, [uint8([165 90]), body, uint8(bitand(crc, 255)), uint8(bitshift(crc, -8))], 'uint8');
frames = struct('type', {}, 'payload', {});
tEnd = tic; limit = 3;
while toc(tEnd) < limit
    n = sp.NumBytesAvailable;
    if n == 0, pause(0.002); continue; end
    [fr, st.rest] = parseTelemetry([st.rest, read(sp, n, 'uint8')]);
    for f = fr
        if f.type == 14 && numel(f.payload) >= 4 && f.payload(1) == op && f.payload(2) == st.seq
            res = f.payload(3);
            if res == 5, limit = 300; continue; end                 % IN_PROGRESS (erase)
            if res ~= 0
                error('acwingLogDownload:ack', 'command 0x%02X refused: result %d, reason %d (8 = armed)', ...
                      op, res, f.payload(4));
            end
            return;
        end
        frames(end+1) = f; %#ok<AGROW>
        tEnd = tic;
    end
end
error('acwingLogDownload:timeout', 'no answer to 0x%02X: is this the FC USB port?', op);
end
