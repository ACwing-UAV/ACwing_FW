function [frames, rest] = parseTelemetry(buf, skipTypes)
%PARSETELEMETRY  Extracts the complete, valid frames from a byte buffer.
%
%   [frames, rest] = parseTelemetry(buf, skipTypes)
%   buf    : uint8 row, received bytes (possibly concatenated with the rest
%            of the previous call)
%   frames : struct array with fields .type (uint8) and .payload (uint8 row)
%   rest   : bytes not yet consumed (incomplete frame at the tail)
%
%   Format (see App/telemetry/telemetry.h):
%   0xA5 0x5A | TYPE | LEN | PAYLOAD | CRC16 LSB MSB, CRC over TYPE..PAYLOAD.
%   A wrong CRC shifts by one byte and re-synchronises.
%
%   skipTypes (optional): types to SKIP without computing the CRC (e.g. 4 =
%   REPLAY in the viewer, which already stores them raw in the .bin). The CRC in
%   MATLAB costs ~1 us per byte: at ~44 kB/s skipping REPLAY halves the work.
%   A skipped frame is accepted only if a new sync follows immediately.

if nargin < 2, skipTypes = []; end
frames = struct('type', {}, 'payload', {});
buf = uint8(buf(:)).';
n = numel(buf);
i = 1;
while i + 5 <= n
    if buf(i) ~= 165 || buf(i+1) ~= 90
        j = find(buf(i+1:n-1) == 165 & buf(i+2:n) == 90, 1);
        if isempty(j), i = n; break; end
        i = i + j;
        continue;
    end
    len   = double(buf(i+3));
    total = len + 6;
    if i + total - 1 > n, break; end               % incomplete frame: wait
    if ~isempty(skipTypes) && any(double(buf(i+2)) == skipTypes)
        j = i + total;                             % next sync expected here
        if j + 1 > n, break; end                   % wait until it arrives
        if buf(j) == 165 && buf(j+1) == 90
            i = j; continue;
        end
    end
    body  = buf(i+2 : i+3+len);
    crcRx = double(buf(i+4+len)) + 256*double(buf(i+5+len));
    if crc16ccitt(body) == crcRx
        frames(end+1) = struct('type', buf(i+2), 'payload', buf(i+4 : i+3+len)); %#ok<AGROW>
        i = i + total;
    else
        i = i + 1;                                 % false sync or corrupted data
    end
end
rest = buf(i:end);
end
