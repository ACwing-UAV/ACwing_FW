function F = acwingLogDecode(src)
%ACWINGLOGDECODE  Decodes an ACwing flight log (format 1) downloaded from the FC.
%
%   F = acwingLogDecode('acwing_flight_00012.bin')
%   F = acwingLogDecode(bytes)          % uint8 vector, sectors concatenated
%
%   F.info    firmware, board, start time
%   F.att     .t [s], .q 4xN [w x y z], .gyro 3xN [rad/s], .acc 3xN [g]       (50 Hz)
%   F.vert    .t, .h, .vz, .h_sp, .vz_sp [m, m/s], .thrust 0..1, .tof [m],
%             .flow_q, .baro_pa, .flags                                       (50 Hz)
%   F.flight  .t, .armed, .mode, .autom, .failsafe, .flags, .sticks 4xN, .silence_ms (10 Hz)
%   F.event   .t, .level, .code, .arg, .name (cell)
%   F.est     .t, .gyro_bias 3xN [rad/s], .att_unc_deg, .vib_mg, .loop_hz,
%             .jitter_us, .cpu_pct, .mag_interf_pct                          (10 Hz)
%   F.power   .t, .vbat, .current [A], .used_mah, .cell, .cell_rest [V], .pct,
%             .level, .r_cell [ohm], .cells, .present                        (10 Hz)
%   F.drop    .t, .lost, .source
%   F.sectors, F.bad (rejected sectors/records), F.unknown
%
%   Format: App/log/log_format.h. Python equivalent: acwing_tools/flightlog.py
%   (decode_log); both give the same numbers. Download the logs with
%   acwingLogDownload or "acwing-tools log get".

if ischar(src) || isstring(src)
    fid = fopen(src, 'r');
    if fid < 0, error('acwingLogDecode:file', 'cannot open %s', src); end
    d = fread(fid, inf, 'uint8=>uint8').';
    fclose(fid);
else
    d = uint8(src(:)).';
end

SECTOR = 4096; MAGIC = uint32(hex2dec('474C4341'));         % "ACLG"
% record type -> name, field list {name, type, count}
R = containers.Map('KeyType', 'double', 'ValueType', 'any');
R(1)   = {'info',   {'fw','uint8',3; 'hw','uint8',1; 'format','uint8',1; 't0_ms','uint32',1; 'sensors','uint16',1; 'name','uint8',16}};
R(16)  = {'att',    {'t_ms','uint32',1; 'q','int16',4; 'g','int16',3; 'a','int16',3}};
R(17)  = {'vert',   {'t_ms','uint32',1; 'h','int32',1; 'vz','int16',1; 'hsp','int32',1; 'vzsp','int16',1; 'thr','uint8',1; ...
                     'tof','uint16',1; 'fq','uint8',1; 'baro','uint32',1; 'flags','uint8',1}};
R(18)  = {'flight', {'t_ms','uint32',1; 'armed','uint8',1; 'mode','uint8',1; 'autom','uint8',1; 'failsafe','uint8',1; ...
                     'flags','uint8',1; 'st','int8',4; 'sil','uint16',1}};
R(19)  = {'event',  {'t_ms','uint32',1; 'level','uint8',1; 'code','uint16',1; 'arg','int32',1}};
R(20)  = {'est',    {'t_ms','uint32',1; 'b','int16',3; 'unc','uint16',1; 'vib','uint16',1; 'hz','uint16',1; ...
                     'jit','uint16',1; 'cpu','uint8',1; 'mi','uint8',1}};
R(32)  = {'power',  {'t_ms','uint32',1; 'vbat','uint16',1; 'cur','int16',1; 'used','uint16',1; 'cell','uint16',1; ...
                     'rest','uint16',1; 'pct','uint8',1; 'level','uint8',1; 'r','uint16',1; 'cells','uint8',1}};
R(127) = {'drop',   {'t_ms','uint32',1; 'lost','uint16',1; 'source','uint8',1}};

types = cell2mat(keys(R));
raw = struct();
for t = types, v = R(t); raw.(v{1}) = {}; end
bad = 0; unknown = 0; sectors = 0;
for s = 0:SECTOR:numel(d) - SECTOR
    h = d(s+1:s+20);
    if typecast(h(1:4), 'uint32') ~= MAGIC || h(17) ~= 1 || ...
            double(typecast(h(19:20), 'uint16')) ~= crc16ccitt(h(1:18))
        bad = bad + 1;
        continue;
    end
    sectors = sectors + 1;
    i = s + 21; e = s + SECTOR;
    while i + 2 <= e && d(i) ~= 255
        t = double(d(i)); n = double(d(i+1));
        if i + 2 + n > e || crc8(d(i:i+1+n)) ~= d(i+2+n)
            bad = bad + 1;                                   % cut record: skip the rest of the sector
            break;
        end
        if isKey(R, t)
            v = R(t);
            if recSize(v{2}) == n
                raw.(v{1}){end+1} = d(i+2:i+1+n);
            else
                unknown = unknown + 1;
            end
        else
            unknown = unknown + 1;
        end
        i = i + 3 + n;
    end
end

F = struct('sectors', sectors, 'bad', bad, 'unknown', unknown, 'info', []);
for t = types
    v = R(t);
    C = unpack(raw.(v{1}), v{2});
    F = scaleRecord(F, v{1}, C);
end
end

% ------------------------------------------------------------------------
function n = recSize(fields)
n = 0;
for k = 1:size(fields, 1)
    n = n + fields{k,3} * bytesOf(fields{k,2});
end
end

function b = bytesOf(ty)
switch ty
    case {'uint8','int8'},   b = 1;
    case {'uint16','int16'}, b = 2;
    otherwise,               b = 4;
end
end

function C = unpack(rows, fields)
N = numel(rows);
C = struct();
off = 0;
M = zeros(N, 0, 'uint8');
if N, M = cat(1, rows{:}); end
for k = 1:size(fields, 1)
    nb = bytesOf(fields{k,2}); cnt = fields{k,3};
    v = zeros(cnt, N);
    for j = 1:cnt
        if N
            b = M(:, off + (j-1)*nb + (1:nb)).';
            v(j, :) = double(typecast(b(:).', fields{k,2}));
        end
    end
    C.(fields{k,1}) = v;
    off = off + nb * cnt;
end
C.n = N;
end

function F = scaleRecord(F, name, C)
if strcmp(name, 'info')
    if C.n
        F.info = struct('fw', sprintf('%d.%d.%d', C.fw(:,1)), 'hw_rev', char(C.hw(1)), ...
            'format', C.format(1), 't0_ms', C.t0_ms(1), 'sensors', C.sensors(1), ...
            'name', deblank(char(C.name(C.name(:,1) > 0, 1).')));
    end
    return;
end
S = struct('n', C.n, 't', C.t_ms * 1e-3);
switch name
    case 'att'
        S.q = C.q / 32767; S.gyro = C.g * 1e-3; S.acc = C.a * 1e-3;
    case 'vert'
        S.h = C.h * 1e-3; S.vz = C.vz * 1e-3; S.h_sp = C.hsp * 1e-3; S.vz_sp = C.vzsp * 1e-3;
        S.thrust = C.thr / 255;
        S.tof = C.tof * 1e-3; S.tof(C.tof == 65535) = NaN;
        S.flow_q = C.fq; S.flow_q(C.fq == 255) = NaN;
        S.baro_pa = C.baro; S.baro_pa(C.baro == 0) = NaN;
        S.flags = C.flags;
    case 'flight'
        S.armed = C.armed; S.mode = C.mode; S.autom = C.autom; S.failsafe = C.failsafe;
        S.flags = C.flags; S.sticks = C.st / 100; S.silence_ms = C.sil;
    case 'event'
        S.level = C.level; S.code = C.code; S.arg = C.arg;
        S.name = arrayfun(@eventName, C.code, 'UniformOutput', false);
    case 'est'
        S.gyro_bias = C.b * 1e-4; S.att_unc_deg = C.unc / 100; S.vib_mg = C.vib;
        S.loop_hz = C.hz; S.jitter_us = C.jit; S.cpu_pct = C.cpu; S.mag_interf_pct = C.mi;
    case 'power'
        S.vbat = C.vbat * 1e-3; S.current = C.cur * 1e-2; S.used_mah = C.used;
        S.cell = C.cell * 1e-3; S.cell_rest = C.rest * 1e-3; S.pct = C.pct; S.level = C.level;
        S.r_cell = C.r * 1e-3; S.cells = mod(C.cells, 128); S.present = double(C.cells >= 128);
    case 'drop'
        S.lost = C.lost; S.source = C.source;
end
F.(name) = S;
end

function s = eventName(code)
names = {1,'ARMED'; 2,'DISARMED'; 3,'TAKEOFF_STARTED'; 4,'LANDED'; 5,'MODE_CHANGED'; 6,'RTH_STARTED'; ...
    7,'HOME_SET'; 8,'TAKEOFF_ABORTED'; 20,'FAILSAFE_APP_LINK'; 21,'FAILSAFE_RADIO'; 22,'FAILSAFE_BATTERY'; ...
    23,'GEOFENCE_BREACH'; 24,'GPS_LOST'; 25,'FLOW_LOST'; 30,'BATTERY_LOW'; 31,'BATTERY_CRITICAL'; ...
    32,'BATTERY_EMERGENCY'; 33,'BATTERY_MONITOR_LOST'; 40,'MAG_INTERFERENCE'; 41,'HIGH_VIBRATION'; ...
    42,'MOTOR_HOT'; 43,'RADIO_OVERRIDE'; 50,'CALIB_DONE'; 51,'CALIB_FAILED'; 60,'PARAMS_SAVED'; 70,'MOTOR_KILLED'};
k = find(cell2mat(names(:,1)) == code, 1);
if isempty(k), s = sprintf('EVENT_%d', code); else, s = names{k,2}; end
end

function c = crc8(b)
%CRC-8/SMBUS (poly 0x07, init 0), as log_crc8() in C
c = uint8(0);
for x = uint8(b)
    c = bitxor(c, x);
    for k = 1:8
        if bitand(c, 128)
            c = bitxor(bitshift(c, 1), uint8(7));
        else
            c = bitshift(c, 1);
        end
    end
end
end
