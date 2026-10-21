%INITPATH  Adds every folder of the MATLAB tools to the path.
%   Run once per MATLAB session, from anywhere:
%       run('<repo>/Matlab/initPath.m')
%   Paths are resolved relative to this file, so the current folder does not matter.
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'Telemetry'));
addpath(fullfile(root, 'Viewer'));
addpath(fullfile(root, 'MagCalib'));
addpath(fullfile(root, 'FiltersComparison'));                          % capture / sniffer / comparison
addpath(fullfile(root, 'FiltersComparison', 'IEKF_originalVersion'));  % ORIGINAL filter: iekfAttitude.m, expSO3.m, ...
addpath(fullfile(root, 'FiltersComparison', 'IEKF_embeddedVersion'));  % extended filter (C mirror)
fprintf('IEKF tools on the path (reference filter: %s)\n', which('iekfAttitude'));
clear root
