function [Rest, bEst, Phist] = iekfAttitude(gyro, acc, mag, dt, gRef, mRef, opts)
%IEKFATTITUDE  Invariant Extended Kalman Filter for attitude estimation on SO(3).
%
%   [Rest, bEst, Phist] = iekfAttitude(gyro, acc, mag, dt, gRef, mRef, opts)
%
%   Estimates the orientation of an IMU/MARG unit by fusing a rate gyroscope
%   (propagation) with accelerometer and magnetometer vector observations
%   (correction), using a left-invariant error formulation on the Lie group
%   SO(3).
%
%   THEORY
%   ------
%   The state R lives on SO(3), not in a vector space, so the estimation error
%   is NOT defined as a subtraction. Instead it is defined through the group
%   action, as a body-frame ("left-invariant") error:
%
%       R_true = R_hat * expSO3(delta)      =>    delta = logSO3(R_hat' * R_true)
%
%   delta is an element of the Lie algebra so(3) ~ R^3, which IS a vector
%   space: all the standard Gaussian/Kalman machinery (means, covariances,
%   linear updates) is applied there, consistently.
%
%   Because the IMU propagation is group-affine, the error dynamics are
%   autonomous ("log-linear"): the transition Jacobian does not depend on the
%   estimated state R_hat, only on the measured angular rate. This is the key
%   property that separates the IEKF from the classical multiplicative EKF and
%   gives it much stronger convergence behaviour from poor initialisations.
%
%   Error-state (with bias estimation enabled):
%       x = [ delta ; beta ],   beta = b_true - b_hat   (gyro bias error)
%
%       delta_dot = -skew(omega_hat)*delta - beta - n_g
%       beta_dot  = n_b
%
%   Measurement model (a known world-frame vector v observed in body frame):
%       y = R' * v + noise,   y_hat = R_hat' * v
%       y - y_hat  ~=  skew(y_hat) * delta       => H = [ skew(y_hat) , 0 ]
%
%   INPUTS
%   ------
%     gyro  3xN  angular rate in the body frame          [rad/s]
%     acc   3xN  accelerometer, body frame               [any unit, normalised internally]
%     mag   3xN  magnetometer, body frame                [any unit, normalised internally]
%                pass [] to run accelerometer-only (attitude unobservable in yaw)
%     dt    1x1 or 1xN   sample period(s)                [s]
%     gRef  3x1  normalised gravity direction in the world frame
%     mRef  3x1  normalised magnetic field direction in the world frame
%     opts  struct (all fields optional):
%            .R0             3x3   initial attitude estimate            [default eye(3)]
%            .b0             3x1   initial gyro bias estimate           [default zeros]
%            .estimateBias   bool  augment state with gyro bias         [default true]
%            .P0             1x1 or nxn  initial covariance             [default 1.0]
%            .sigmaGyro      1x1   gyro white noise std      [rad/s]    [default deg2rad(0.5)]
%            .sigmaBias      1x1   bias random-walk std      [rad/s^2]  [default 1e-3]
%            .sigmaAcc       1x1   accel meas. noise std (normalised)   [default 0.05]
%            .sigmaMag       1x1   mag   meas. noise std (normalised)   [default 0.05]
%            .adaptive       bool  inflate R when |acc|/|mag| deviate   [default true]
%            .accTol         1x1   relative tolerance on |acc|          [default 0.05]
%            .magTol         1x1   relative tolerance on |mag|          [default 0.05]
%            .tiltCompensatedMag  bool  see below                       [default false]
%
%   TILT-COMPENSATED MAGNETOMETER UPDATE
%   -------------------------------------
%   With a fixed world-frame mRef, the magnetometer update constrains the full
%   3-vector R'*mRef, which implicitly also constrains the gravity/field
%   inclination angle. Any small inconsistency between gRef and mRef (noise in
%   the calibration window, hard/soft-iron residuals) then leaks into the
%   estimate as a spurious, tilt-dependent yaw error: exactly the "yaw moves
%   during a pure roll" symptom.
%
%   Setting opts.tiltCompensatedMag = true replaces the fixed-mRef prediction
%   with the classical tilt-compensated-compass construction (the same trick
%   used inside mahonyAHRS/madgwickAHRS): the current magnetometer reading is
%   rotated into the world frame with the CURRENT estimate, its horizontal
%   component is collapsed onto a single reference axis, and only THAT is
%   compared back against a prediction. The vertical (inclination) component
%   is taken directly from the measurement itself, so it never enters the
%   innovation. Only the yaw-sensitive part of the field is left to correct
%   the estimate; mRef is not used at all in this mode.
%
%   OUTPUTS
%   -------
%     Rest   3x3xN  estimated rotation matrices (body -> world)
%     bEst   3xN    estimated gyro bias                            [rad/s]
%     Phist  nxnxN  error-state covariance history
%
%   See also EXPSO3, LOGSO3, SKEWSYM, MAHONYAHRS, MADGWICKAHRS.
 
% ----------------------------- options ----------------------------------
if nargin < 7, opts = struct(); end
if ~isfield(opts,'R0'),           opts.R0           = eye(3);        end
if ~isfield(opts,'b0'),           opts.b0           = zeros(3,1);    end
if ~isfield(opts,'estimateBias'), opts.estimateBias = true;          end
if ~isfield(opts,'P0'),           opts.P0           = 1.0;           end
if ~isfield(opts,'sigmaGyro'),    opts.sigmaGyro    = deg2rad(0.5);  end
if ~isfield(opts,'sigmaBias'),    opts.sigmaBias    = 1e-3;          end
if ~isfield(opts,'sigmaAcc'),     opts.sigmaAcc     = 0.05;          end
if ~isfield(opts,'sigmaMag'),     opts.sigmaMag     = 0.05;          end
if ~isfield(opts,'adaptive'),     opts.adaptive     = true;          end
if ~isfield(opts,'accTol'),       opts.accTol       = 0.05;          end
if ~isfield(opts,'magTol'),       opts.magTol       = 0.05;          end
if ~isfield(opts,'tiltCompensatedMag'), opts.tiltCompensatedMag = false; end
 
N       = size(gyro, 2);
useMag  = ~isempty(mag);
useBias = opts.estimateBias;
n       = 3 + 3*useBias;
 
if isscalar(dt), dtv = repmat(dt, 1, N); else, dtv = dt(:).'; end
 
gRef = gRef(:) / norm(gRef);
if useMag, mRef = mRef(:) / norm(mRef); end
 
% ----------------------------- init -------------------------------------
Rest  = zeros(3,3,N);
bEst  = zeros(3,N);
Phist = zeros(n,n,N);
 
R = opts.R0;
b = opts.b0(:);
 
if isscalar(opts.P0), P = eye(n)*opts.P0; else, P = opts.P0; end
 
Rest(:,:,1) = R;
bEst(:,1)   = b;
Phist(:,:,1)= P;
 
% Process noise (continuous densities, scaled by dt in the loop)
Qg = (opts.sigmaGyro^2) * eye(3);
Qb = (opts.sigmaBias^2) * eye(3);
 
% Nominal measurement covariances
Racc0 = (opts.sigmaAcc^2) * eye(3);
Rmag0 = (opts.sigmaMag^2) * eye(3);
 
% Nominal magnitudes used by the adaptive gating (from the first valid sample)
nomAcc = norm(acc(:,1));
if nomAcc < eps, nomAcc = 1; end
if useMag
    nomMag = norm(mag(:,1));
    if nomMag < eps, nomMag = 1; end
end
 
% ----------------------------- main loop --------------------------------
for k = 2:N
 
    h  = dtv(k);
    if ~isfinite(h) || h <= 0, h = median(dtv(isfinite(dtv) & dtv > 0)); end
 
    % ---------- PROPAGATION ----------
    % Bias-corrected angular rate, then exact integration on the group.
    om = gyro(:,k-1) - b;
    R  = R * expSO3(om*h);
 
    % Error-state transition. NOTE: F depends only on the MEASURED rate,
    % never on the estimated attitude R -- this is the group-affine property.
    if useBias
        F = eye(6);
        F(1:3,1:3) = expSO3(-om*h);
        F(1:3,4:6) = -eye(3)*h;
        Q = blkdiag(Qg*h, Qb*h);
    else
        F = expSO3(-om*h);
        Q = Qg*h;
    end
    P = F*P*F.' + Q;
 
    % ---------- ACCELEROMETER UPDATE ----------
    na = norm(acc(:,k));
    if na > eps
        y    = acc(:,k) / na;
        yhat = R.' * gRef;
 
        Racc = Racc0;
        if opts.adaptive
            % When the measured magnitude departs from its nominal value the
            % vector is contaminated (external acceleration / magnetic
            % disturbance): trust it less instead of rejecting it outright.
            dev  = abs(na - nomAcc) / nomAcc;
            Racc = Racc0 * (1 + (dev/opts.accTol)^2);
        end
 
        H = zeros(3,n);
        H(:,1:3) = skewSym(yhat);
 
        [R, b, P] = applyUpdate(R, b, P, H, y - yhat, Racc, n, useBias);
    end
 
    % ---------- MAGNETOMETER UPDATE ----------
    if useMag
        nm = norm(mag(:,k));
        if nm > eps
            y = mag(:,k) / nm;
 
            if opts.tiltCompensatedMag
                % Tilt-compensated construction: rotate the CURRENT reading
                % into the world frame with the CURRENT estimate, collapse its
                % horizontal part onto a single axis, and predict back from
                % that. The vertical/inclination component is taken verbatim
                % from the measurement, so it cancels exactly in the
                % innovation and cannot leak into the correction. mRef is
                % unused in this branch.
                hw     = R * y;
                bMagW  = [norm(hw(1:2)); 0; hw(3)];
                yhat   = R.' * bMagW;
            else
                yhat = R.' * mRef;
            end
 
            Rmag = Rmag0;
            if opts.adaptive
                dev  = abs(nm - nomMag) / nomMag;
                Rmag = Rmag0 * (1 + (dev/opts.magTol)^2);
            end
 
            H = zeros(3,n);
            H(:,1:3) = skewSym(yhat);
 
            [R, b, P] = applyUpdate(R, b, P, H, y - yhat, Rmag, n, useBias);
        end
    end
 
    Rest(:,:,k)  = R;
    bEst(:,k)    = b;
    Phist(:,:,k) = P;
end
end
 
% =========================================================================
function [R, b, P] = applyUpdate(R, b, P, H, innov, Rm, n, useBias)
%APPLYUPDATE  One Kalman correction, injected back onto the group.
 
S = H*P*H.' + Rm;
K = (P*H.') / S;              % right division: numerically better than inv(S)
dx = K*innov;
 
% Inject the attitude correction through the exponential map (retraction onto
% SO(3)) -- this is what keeps the estimate a valid rotation at all times.
R = R * expSO3(dx(1:3));
 
if useBias
    % dx(4:6) estimates beta = b_true - b_hat, hence the PLUS sign here.
    b = b + dx(4:6);
end
 
% Joseph form: preserves symmetry and positive-definiteness.
IKH = eye(n) - K*H;
P   = IKH*P*IKH.' + K*Rm*K.';
P   = (P + P.')/2;
end