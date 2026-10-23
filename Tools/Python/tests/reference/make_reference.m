%MAKE_REFERENCE  Reference outputs of the MATLAB filters for the Python parity tests.
%   Run from Tools/Python/tests/reference in MATLAB or Octave, after
%   make_inputs.py. Writes ../data/filter_reference.mat and
%   ../data/compare_reference.mat. The three filter configurations below must
%   be kept identical to tests/test_filters.py.
here = fileparts(mfilename('fullpath'));
run(fullfile(here, '..', '..', '..', '..', 'Matlab', 'initPath.m'));
in = load(fullfile(here, '..', 'data', 'filter_inputs.mat'));
gRef = [0;0;1]; mRef = [0.5;0;-0.85];
o = struct('R0', quat2rotmLocal(in.q0), 'b0', in.b0(:), ...
           'P0', diag([1e-2*[1 1 1], 1e-4*[1 1 1]]), 'sigmaAcc', 0.03, ...
           'adaptive', true, 'tiltCompensatedMag', false);
[Ro, bo, Po] = iekfAttitude(in.gyro, in.acc, in.mag, in.dt, gRef, mRef, o);
oe = o; oe.accNisGate = 4; oe.magNisGate = 9; oe.inflTau = 0.5; oe.dipTol = deg2rad(5);
oe.lockoutTime = 0.1; oe.nomAcc = 1.0; oe.nomMag = 0.9; oe.inflMax = 100;
oe.accTrusted = in.accTr; oe.magExtraVar = in.magXV;
[Rx, bx, Px, ix] = iekfAttitudeEmb(in.gyro, in.acc, in.mag, in.dt, gRef, mRef, oe);
oe2 = oe; oe2.decouple = true; oe2.tiltCompensatedMag = true; oe2.estimateBias = false; oe2.P0 = 0.01;
[Ry, by, Py, iy] = iekfAttitudeEmb(in.gyro, in.acc, in.mag, in.dt, gRef, mRef, oe2);
lockouts = ix.lockouts; accInflMax = ix.accInflMax;
save('-v7', fullfile(here, '..', 'data', 'filter_reference.mat'), ...
     'Ro', 'bo', 'Po', 'Rx', 'bx', 'Px', 'Ry', 'by', 'Py', 'lockouts', 'accInflMax');

for c = {'capture_mode1', 'capture_mode0'}
    out = compareEmbeddedVsMatlab(fullfile(here, '..', 'data', [c{1} '.bin']), false);
    r.(c{1}).dAng = out.dAng; r.(c{1}).dBias = out.dBias; r.(c{1}).dP = out.dP;
    r.(c{1}).ok = double(out.ok); r.(c{1}).eulRef = out.eulRef;
    if ~isempty(out.extended)
        r.(c{1}).extdAng = out.extended.dAng; r.(c{1}).extdBias = out.extended.dBias;
        r.(c{1}).extdP = out.extended.dP;
    else
        r.(c{1}).adaptdAng = out.adapted.dAng;
    end
end
save('-v7', fullfile(here, '..', 'data', 'compare_reference.mat'), '-struct', 'r');
fprintf('reference written (lockouts %d, accInflMax %g)\n', lockouts, accInflMax);
