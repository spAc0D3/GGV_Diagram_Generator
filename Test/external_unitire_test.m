clc; close all;

% Add the tire model subfolder to the path.
root = fileparts(mfilename('fullpath'));
projRoot = fullfile(root, '..');
addpath(fullfile(projRoot, 'external/FSAE-VD-Personal-Scripts/Yaw Dynamics/Tire Model'));

tireData = load(fullfile(projRoot, 'external/FSAE-VD-Personal-Scripts/Yaw Dynamics/Tire Model/Hoosier_R20_Combined_Simple.mat'));
tire = tireData.tire;

a = unitire_simple_solve(0, 0.2, 1000, 10, tire, struct());