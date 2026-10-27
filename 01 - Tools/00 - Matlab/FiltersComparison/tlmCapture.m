function binFile = tlmCapture(port, duration, baud, outDir)
%TLMCAPTURE  Records the raw telemetry to a file, with no decoding and no graphics.
%
%   binFile = tlmCapture("COM6", 60)            % 60 s at 921600 baud
%   binFile = tlmCapture("COM6", 60, 921600, "C:\path")
%
%   Used for the comparison with the original MATLAB filter (APP_TLM_REPLAY = 1):
%   at ~44 kB/s the real-time viewer cannot keep up, while writing the bytes to
%   disk costs almost nothing. The file is analysed afterwards:
%       compareEmbeddedVsMatlab(binFile)
%
%   Procedure: start tlmCapture, THEN reset the board, keep it still for 2-4 s,
%   then move it. Window 0 of the trace arrives right after the calibration.

if nargin < 2 || isempty(duration), duration = 60; end
if nargin < 3 || isempty(baud),     baud     = 921600; end
if nargin < 4 || isempty(outDir),   outDir   = pwd; end

binFile = fullfile(outDir, sprintf('acwing_cap_%s.bin', datestr(now, 'yyyymmdd_HHMMSS'))); %#ok<TNOW1,DATST>
fid = fopen(binFile, 'w');
if fid < 0, error('Cannot create %s', binFile); end
dev = serialport(port, baud, "Timeout", 1);
flush(dev);
fprintf('Recording to %s for %d s: reset the board NOW.\n', binFile, duration);

nTot = 0; t0 = tic; tNext = 5; nHalf = NaN;
try
    while toc(t0) < duration
        nb = dev.NumBytesAvailable;
        if nb > 0
            fwrite(fid, uint8(read(dev, nb, "uint8")), 'uint8');
            nTot = nTot + nb;
        else
            pause(0.01);
        end
        if isnan(nHalf) && toc(t0) > duration/2, nHalf = nTot; tHalf = toc(t0); end
        if toc(t0) > tNext
            fprintf('  %3.0f s: %8d bytes (%.1f kB/s)\n', toc(t0), nTot, nTot/toc(t0)/1000);
            tNext = tNext + 5;
        end
    end
catch err
    fclose(fid); clear dev
    rethrow(err);
end
fclose(fid);
clear dev
fprintf('Done: %d bytes (%.1f kB/s on average).\n  compareEmbeddedVsMatlab(''%s'')\n', ...
        nTot, nTot/duration/1000, binFile);
% Rate measured only over the second half: the first half includes waiting for
% the reset and the 2 s calibration, during which only STATUS (~0.6 kB/s) is sent.
rate2 = (nTot - nHalf) / (duration - tHalf);
fprintf('Rate over the second half: %.1f kB/s (expected ~44 kB/s with REPLAY)\n', rate2/1000);
if rate2 < 35e3
    warning(['Second half below 35 kB/s: REPLAY not enabled, filter not started, or ' ...
             'baud rate different from %d (check USART1 in CubeMX).'], baud);
end
end
