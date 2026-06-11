% GGV Diagram Generator

clc; close all;

% Add the tire model subfolder to the path.
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'external/FSAE-VD-Personal-Scripts/Yaw Dynamics/Tire Model'));

%% 1. Vehicle Data Loading

fprintf('Vehicle Data Loading...\n');

g     = 9.81;
rho_air = 1.2;
% read basic params
rawV = readBasicParamsCSV(fullfile(root, 'Vehicle_Data/Vehicle_BasicParams.csv'));
Mass_Total = str2double(rawV('m_fsae'));
Mass_FrontRatio = str2double(rawV('m_f_Ratio'));
WB = str2double(rawV('WheelBase')) / 1000;
TW_F = str2double(rawV('TrackWidth_f'))/1000;
TW_R = str2double(rawV('TrackWidth_r'))/1000;
CoG_h = str2double(rawV('CoG_h')) / 1000;
R_e = str2double(rawV('R_e')) / 1000;
RH_std = str2double(rawV('rh_std')) / 1000;
Mech_drag = str2double(rawV('Mechanical_drag'));
TransEff = str2double(rawV('TransmissionEfficiency'));
% support fractional gear ratio like "43/11"
grStr= rawV('GearRatio');
if contains(grStr,'/')
    s = strsplit(grStr,'/'); gr = str2double(s{1})/str2double(s{2});
else
    gr = str2double(grStr);
end

% read drivetrain params
fid_drivetrain = fopen(fullfile(root, 'Vehicle_Data/Vehicle_Motor_PeakTorque_EMRAX228_80kW.csv'));
rpmLine  = strsplit(strtrim(fgetl(fid_drivetrain)), ',');
torqLine = strsplit(strtrim(fgetl(fid_drivetrain)), ',');
fclose(fid_drivetrain);
motor_rpm_range = cellfun(@str2double, rpmLine(2:end));
motor_Tq_range  = cellfun(@str2double, torqLine(2:end));

% read aero maps (ride height + yaw/side-slip characteristics)
[aero_rhF, aero_rhR, aero_CL, aero_CD, aero_COP, ...
 aero_yawAngle, aero_yawCL, aero_yawCD, aero_yawCOP] = readAeroCSV(...
    fullfile(root, 'Vehicle_Data/Vehicle_AeroMap_RideHeight.csv'), ...
    fullfile(root, 'Vehicle_Data/Vehicle_AeroMap_Yaw.csv'));

%% 2. Pre-calculations

Fz0   = Mass_Total * g;
Fz0_f = Fz0 * Mass_FrontRatio;
Fz0_r = Fz0 * (1 - Mass_FrontRatio);

V_range = 0:1:max(motor_rpm_range)*pi*R_e/30/gr;

% load UniTire simple tire model
tireData = load(fullfile(root, 'external/FSAE-VD-Personal-Scripts/Yaw Dynamics/Tire Model/unitire_simple_fit.mat'));
tire = tireData.tire;

%% 3. Main Loop: GGV Diagram Calculation

% initailize tire model input parameters and storage
% slip angle α, slip ratio κ, wheel hub angular speed Omega, Vertical Load F_z
% rows: [FL; FR; RL; RR] columns: [α, κ, F_z, Omega] 
alpha = 0; kappa = 0; F_z = 0; Omega = 0;
% initialize WheelParams [α, κ, F_z, Omega]
WheelParams = zeros(4,4);
a_x_store    = zeros(size(V_range));
Fx_motor_store = zeros(size(V_range));  % motor tractive force at wheel

for i = 1:length(V_range)
    V_x = V_range(i);

    % longitudinal acceleration calculation loop
    WheelParams(:, 1) = 0;
    WheelParams([1,2], 2) = 0;
    a_x_max = 0;
    for kappa = linspace(0, 0.25, 26)
        WheelParams([3,4], 2) = kappa;
        WheelParams([3,4], 4) = V_x * (1 + kappa) / R_e;
        % Iteratively solve: tire forces ↔ load transfer
        Fz_f = Fz0_f; Fz_r = Fz0_r; a_x_prev = 0;
        for iter = 1:20
            WheelParams([1,2], 3) = Fz_f/2;
            WheelParams([3,4], 3) = Fz_r/2;
            a = unitire_simple_solve(...
                WheelParams(:,1), WheelParams(:,2), ...
                WheelParams(:,3), WheelParams(:,4), tire, struct());
            Fx_tractive = a.F_tire(3,1) + a.F_tire(4,1);
            % motor rpm & torque (based on driven wheels)
            motor_rpm = max(WheelParams(3,4), WheelParams(4,4)) * 60/(2*pi) * gr;
            motor_rpm = max(min(motor_rpm, max(motor_rpm_range)), min(motor_rpm_range));
            motor_tq = interp1(motor_rpm_range, motor_Tq_range, motor_rpm, 'linear');
            Fx_motor = motor_tq * gr / R_e * TransEff;
            % limit tractive force by motor capability
            Fx_drive = min(Fx_tractive, Fx_motor);
            % Net acceleration = (drive - mechanical drag) / mass
            a_x_new = (Fx_drive - Mech_drag) / Mass_Total;
            % longitudinal load transfer uses the motor-limited acceleration
            dFz = a_x_new * Mass_Total * CoG_h / WB;
            Fz_f_new = Fz0_f - dFz;
            Fz_r_new = Fz0_r + dFz;
            if abs(a_x_new - a_x_prev) < 1e-3
                a_x_eff = max(a_x_new, 0);
                if a_x_eff > a_x_max
                    a_x_max = a_x_eff;
                    Fx_motor_store(i) = Fx_motor;
                end
                break;
            end
            Fz_f = max(Fz_f_new, 0);
            Fz_r = max(Fz_r_new, 0);
            a_x_prev = a_x_new;
        end

        % lateral acceleration calculate loop
        V_y = linspace(0, 0.5*V_x, 20);
        % CoG Slip Angle β
        beta = atan(V_y ./ max(V_x, 1e-4));
        

    end
    a_x_store(i) = a_x_max;
end

%% 4. Plot Longitudinal GGV Result

figure('Name','GGV - Longitudinal','NumberTitle','off','Position',[100 100 900 600]);
V_kmh = V_range * 3.6;

% -- Left y-axis: acceleration (g) --
plot(V_kmh, a_x_store/g, 'b-','LineWidth',2);
ylabel('纵向加速度 a_x (g)','Color','b');
yline(0,'k:');
ylim([0 max(a_x_store/g)*1.1]);

grid on; hold off;
xlabel('车速 (km/h)');
title('GGV Diagram — 纵向加速能力');
xlim([0 max(V_kmh)]);

[max_a_g, idx] = max(a_x_store/g);
fprintf('最大纵向加速度: %.3f g @ %.1f km/h\n', max_a_g, V_kmh(idx));
fprintf('最高车速:        %.1f km/h\n', max(V_kmh));
fprintf('峰值驱动力:      %.0f N\n', max(Fx_motor_store));

%% Functions

function raw = readBasicParamsCSV(filepath)
    fid = fopen(filepath,'r');
    assert(fid~=-1, '无法打开: %s', filepath);
    fgetl(fid); raw = containers.Map;
    while ~feof(fid)
        line = strtrim(fgetl(fid));
        if isempty(line), continue; end
        parts = strsplit(line,',');
        if numel(parts)<2, continue; end
        raw(strtrim(parts{1})) = strtrim(parts{2});
    end
    fclose(fid);
end

function [rhF, rhR, CL, CD, COP, yawAngle, yawCL, yawCD, yawCOP] = readAeroCSV(rhFilepath, yawFilepath)
    % --- ride height aero map ---
    fid = fopen(rhFilepath,'r');
    hdr = strsplit(strtrim(fgetl(fid)),',');
    rhR = cellfun(@str2double, hdr(3:end)) / 1000;
    fgetl(fid);
    nCols = length(rhR);
    fmt = ['%f%*s', repmat('%q',1,nCols)];
    raw = textscan(fid, fmt, 'Delimiter',',');
    fclose(fid);
    rhF = raw{1} / 1000;
    N = length(rhF);
    CL = zeros(N,nCols); CD = zeros(N,nCols); COP = zeros(N,nCols);
    for i = 1:N
        for j = 1:nCols
            tri = str2double(strsplit(raw{j+1}{i},','));
            CL(i,j) = tri(1); CD(i,j)=tri(2); COP(i,j)=tri(3);
        end
    end
    % --- yaw / side-slip aero map ---
    fid2 = fopen(yawFilepath,'r');
    data = textscan(fid2, '%f%f%f%f', 'Delimiter',',', 'HeaderLines',0);
    fclose(fid2);
    yawAngle = data{1};
    yawCL    = data{2};
    yawCD    = data{3};
    yawCOP   = data{4};
end