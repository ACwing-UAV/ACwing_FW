function tlmSniffer(port, baud, duration)
%TLMSNIFFER  Byte-level telemetry debugging.
%
%   tlmSniffer("COM6")              % 921600 baud, 3 s
%   tlmSniffer("COM6", 921600, 5)
%
%   Answers, in order: are bytes arriving? are they valid frames? what does the
%   firmware say about itself (STATUS packet)? Requires parseTelemetry.m,
%   decodeStatus.m, statusDiagnosis.m.

if nargin < 2, baud = 921600; end
if nargin < 3, duration = 3; end

s = serialport(port, baud, "Timeout", 1);
flush(s);
pause(duration);
nb = s.NumBytesAvailable;
buf = uint8([]);
if nb > 0, buf = uint8(read(s, nb, "uint8")); end
clear s

fprintf('\n%s @ %d baud: %d bytes in %.1f s (%.0f B/s)\n', port, baud, nb, duration, nb/duration);
if nb == 0
    fprintf(['No bytes. Check: firmware running (green LED blinking?),\n' ...
             'right COM port ("STMicroelectronics STLink Virtual COM Port"),\n' ...
             'USART1 on PB6/PB7, no other program holding the port open.\n']);
    return;
end

nSync = sum(buf(1:end-1) == 165 & buf(2:end) == 90);
[frames, ~] = parseTelemetry(buf);
types = double([frames.type]);
fprintf('A5 5A sync sequences: %d   valid frames (CRC ok): %d\n', nSync, numel(frames));
fprintf('  STATE: %d   INIT: %d   STATUS: %d   REPLAY: %d\n', ...
        sum(types==1), sum(types==2), sum(types==3), sum(types==4));
fprintf('First bytes: %s\n', sprintf('%02X ', buf(1:min(24,end))));

u = unique(buf);
if numel(u) <= 2
    fprintf(['The stream contains only %d distinct byte values (%s): the DMA keeps\n' ...
             'sending the same byte. In CubeMX, USART1 -> DMA Settings -> USART1_TX:\n' ...
             'tick "Increment Address" on Memory (not on Peripheral). Check the\n' ...
             'same setting on I2C1_RX.\n'], numel(u), sprintf('%02X ', u));
    return;
end
if isempty(frames)
    fprintf(['Bytes present but no valid frame: baud rate differs between firmware\n' ...
             'and MATLAB (check USART1 in CubeMX), or SYSCLK is not 64 MHz.\n']);
    return;
end
if sum(types==3) == 1 && nb < 200
    fprintf(['A single frame: the transmit-complete callback never arrives and the DMA\n' ...
             'stays "busy". Check the USART1 global interrupt and HAL_UART_TxCpltCallback\n' ...
             '-> app_uart_tx_cplt in main.c.\n']);
end

st = frames(find(types == 3, 1, 'last'));
if ~isempty(st)
    S = decodeStatus(st.payload);
    disp(S);
    fprintf('DIAGNOSIS: %s\n', statusDiagnosis(S));
end
end
