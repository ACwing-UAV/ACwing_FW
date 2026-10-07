function R = decodeReplay(p)
%DECODEREPLAY  Payload TYPE 0x04 -> struct (see tlm_build_replay() and est_pipeline.h).
%   R.win, R.overflow, R.ops (struct array: .op 'P'|'A'|'M', .v, .trusted),
%   R.q (4x1), R.b (3x1), R.Pd (6x1), R.accInfl, R.magInfl
p = uint8(p(:)).';
R.win      = double(typecast(p(1:4), 'uint32'));
nops       = double(p(5));
R.overflow = p(6) ~= 0;
tail = double(typecast(p(end-59:end), 'single'));
R.q = tail(1:4).';  R.b = tail(5:7).';  R.Pd = tail(8:13).';
R.accInfl = tail(14);  R.magInfl = tail(15);

R.ops = struct('op', cell(1, nops), 'v', [], 'trusted', false);
i = 7;
for k = 1:nops
    op = char(p(i));
    switch op
        case 'P'
            R.ops(k).v = double(typecast(p(i+1:i+16), 'single')).';  i = i + 17;
        case 'A'
            R.ops(k).v = double(typecast(p(i+1:i+12), 'single')).';
            R.ops(k).trusted = p(i+13) ~= 0;                         i = i + 14;
        case 'M'
            R.ops(k).v = double(typecast(p(i+1:i+16), 'single')).';  i = i + 17;
        otherwise
            error('decodeReplay: unknown operation 0x%02X', double(p(i)));
    end
    R.ops(k).op = op;
end
end
