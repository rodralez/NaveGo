% Example of use of NaveGo.
% 
% Main goal: to compare two INS/GPS systems performances, one using a 
% simulated ADIS16405 IMU and simulated GPS, and another using a 
% simulated ADIS16488 IMU and the same simulated GPS.
%
%   Copyright (C) 2014, Rodrigo Gonzalez, all rights reserved.
%
%   This file is part of NaveGo, an open-source MATLAB toolbox for
%   simulation of integrated navigation systems.
%
%   NaveGo is free software: you can redistribute it and/or modify
%   it under the terms of the GNU Lesser General Public License (LGPL)
%   version 3 as published by the Free Software Foundation.
%
%   This program is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%   GNU Lesser General Public License for more details.
%
%   You should have received a copy of the GNU Lesser General Public
%   License along with this program. If not, see
%   <http://www.gnu.org/licenses/>.
%
% References:
%           R. Gonzalez, J. Giribet, and H. Patiño. NaveGo: a
% simulation framework for low-cost integrated navigation systems,
% Journal of Control Engineering and Applied Informatics, vol. 17,
% issue 2, pp. 110-120, 2015. Eq. 26.
%
%           Analog Devices. ADIS16400/ADIS16405 datasheet. High Precision 
% Tri-Axis Gyroscope, Accelerometer, Magnetometer. Rev. B. 
% http://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16400_16405.pdf
%
%           Analog Devices. ADIS16488 datasheet. Tactical Grade Ten Degrees 
% of Freedom Inertial Sensor. Rev. G. 
% http://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16488.pdf
%
%			Garmin International, Inc. GPS 18x TECHNICAL SPECIFICATIONS.
% Revision D. October 2011. 
% http://static.garmin.com/pumac/GPS_18x_Tech_Specs.pdf
% 
% Version: 006
% Date:    2016/10/25
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

clc
close all
clear
% matlabrc

fprintf('\nStarting simulation ... \n')

%% GLOBAL VARIABLES

global D2R
global R2D

%% CODE EXECUTION PARAMETERS

% Comment any of the following parameters in order to NOT execute a particular portion of code

GPS_DATA  = 'ON';   % Simulate GPS data
IMU1_DATA = 'ON';   % Simulate ADIS16405 IMU data
IMU2_DATA = 'ON';   % Simulate ADIS16488 IMU data

IMU1_INS  = 'ON';   % Execute INS/GPS integration for ADIS16405 IMU
IMU2_INS  = 'ON';   % Execute INS/GPS integration for ADIS16488 IMU

PLOT      = 'ON';   % Plot results.

% If a particular parameter is commented above, it is set by default to 'OFF'.

if (~exist('GPS_DATA','var')),  GPS_DATA  = 'OFF'; end
if (~exist('IMU1_DATA','var')), IMU1_DATA = 'OFF'; end
if (~exist('IMU2_DATA','var')), IMU2_DATA = 'OFF'; end
if (~exist('IMU1_INS','var')),  IMU1_INS = 'OFF'; end
if (~exist('IMU2_INS','var')),  IMU2_INS = 'OFF'; end
if (~exist('PLOT','var')),      PLOT = 'OFF'; end

%% CONVERSION CONSTANTS

G = 9.81;           % Gravity constant, m/s^2
G2MSS = G;          % g to m/s^2
MSS2G = (1/G);      % m/s^2 to g

D2R = (pi/180);     % degrees to radians
R2D = (180/pi);     % radians to degrees

KT2MS = 0.514444;   % knot to m/s
MS2KMH = 3.6;       % m/s to km/h

%% LOAD REFERENCE DATA

fprintf('Loading reference dataset from a trajectory generator... \n')

load ref.mat

% ref.mat contains the reference data structure from which inertial 
% sensors and GPS wil be simulated. It must contain the following fields:

%         t: time vector (seconds).
%       lat: latitude vector (radians).
%       lon: longitude vector (radians).
%         h: altitude vector (meters).
%       vel: NED velocities vectors, [north east down] (meter/s).
%      roll: roll angle vector (radians).
%     pitch: pitch angle vector (radians).
%       yaw: yaw angle vector (radians).
%        kn: number of elements of time vector.
%     DCMnb: Direct Cosine Matrix nav-to-body, with 'kn' rows and 9
%     columns. Each row contains the elements of one matrix ordered by
%     columns as [a11 a21 a31 a12 a22 a32 a13 a23 a33]. Use reshape()
%     built-in MATLAB function to get the original 3x3 matrix
%     (reshape(DCMnb(row,:),3,3)).
%      freq: sampling frequency (Hz).

%% ADIS16405 IMU error profile

ADIS16405.arw       = 2   .* ones(1,3);     % Angle random walks [X Y Z] (deg/root-hour)
ADIS16405.vrw       = 0.2 .* ones(1,3);     % Velocity random walks [X Y Z] (m/s/root-hour)
ADIS16405.gb_fix    = 3 .* ones(1,3);       % Gyro static biases [X Y Z] (deg/s)
ADIS16405.ab_fix    = 50 .* ones(1,3);      % Acc static biases [X Y Z] (mg)
ADIS16405.gb_drift  = 0.007 .* ones(1,3);   % Gyro dynamic biases [X Y Z] (deg/s)
ADIS16405.ab_drift  = 0.2 .* ones(1,3);     % Acc dynamic biases [X Y Z] (mg)
ADIS16405.gcorr     = 100 .* ones(1,3);     % Gyro correlation times [X Y Z] (seconds)
ADIS16405.acorr     = 100 .* ones(1,3);     % Acc correlation times [X Y Z] (seconds)
ADIS16405.freq      = 100;                  % IMU operation frequency [X Y Z] (Hz)
% ADIS16405.m_psd     = 0.066 .* ones(1,3);   % Magnetometer noise [X Y Z] (mgauss/root-Hz)

% ref dataset is used to simulate IMU sensors.

dt = mean(diff(ref.t));               % IMU mean period

imu1 = imu_err_profile(ADIS16405, dt);  % Transform IMU manufacturer units to SI units

imu1.att_init = [1 1 5] .* D2R;         % Initial attitude for matrix P in Kalman filter, [roll pitch yaw] (radians)  
imu1.t = ref.t;                       % IMU time vector
imu1.freq = ref.freq;                 % IMU operation frequency

%% ADIS16488 IMU error profile

ADIS16488.arw = 0.3     .* ones(1,3);       % Angle random walks [X Y Z] (deg/root-hour)
ADIS16488.vrw = 0.029   .* ones(1,3);       % Velocity random walks [X Y Z] (m/s/root-hour)
ADIS16488.gb_fix = 0.2  .* ones(1,3);       % Gyro static biases [X Y Z] (deg/s)
ADIS16488.ab_fix = 16   .* ones(1,3);       % Acc static biases [X Y Z] (mg)
ADIS16488.gb_drift = 6.5/3600  .* ones(1,3);% Gyro dynamic biases [X Y Z] (deg/s)
ADIS16488.ab_drift = 0.1  .* ones(1,3);     % Acc dynamic biases [X Y Z] (mg)
ADIS16488.gcorr = 100 .* ones(1,3);         % Gyro correlation times [X Y Z] (seconds)
ADIS16488.acorr = 100 .* ones(1,3);         % Acc correlation times [X Y Z] (seconds)
ADIS16488.freq = 100;                       % IMU operation frequency [X Y Z] (Hz)
% ADIS16488.m_psd = 0.054 .* ones(1,3);        % Magnetometer noise [X Y Z] (mgauss/root-Hz)

% ref dataset is used to simulate IMU sensors.

dt = mean(diff(ref.t));               % Mean period

imu2 = imu_err_profile(ADIS16488, dt);  % Transform IMU manufacturer error units to SI units.

imu2.att_init = [0.5 0.5 1] .* D2R;     % [roll pitch yaw] Initial attitude for matrix P in Kalman filter
imu2.t = ref.t;                       % IMU time vector
imu2.freq = ref.freq;                 % IMU operation frequency

%% Garmin 5-18 Hz GPS error profile

gps.stdm = [5, 5, 10];                 % GPS positions standard deviations [lat lon h] (meters)
gps.stdv = 0.1 * KT2MS .* ones(1,3);   % GPS velocities standard deviations [Vn Ve Vd] (meters/s)
gps.larm = zeros(3,1);                 % GPS lever arm [X Y Z] (meters)
gps.freq = 5;                          % GPS operation frequency (Hz)

%% SIMULATE GPS

rng('shuffle')                          % Reset pseudo-random seed

if strcmp(GPS_DATA, 'ON')               % If simulation of GPS data is required ...
    
    fprintf('Simulating GPS data... \n')
    
    gps = gps_err_profile(ref.lat(1), ref.h(1), gps); % Transform GPS manufacturer error units to SI units.
    
    [gps, ref_g] = gps_gen(ref, gps);   % Generate GPS dataset from reference dataset.
    % ref_g is the ref dataset
    % resampled at GPS vector time.
    save gps.mat gps
    save ref_g.mat ref_g
    
else
    
    fprintf('Loading GPS data... \n') 
    
    load gps.mat
    load ref_g.mat
end

%% SIMULATE IMU1

rng('shuffle')                  % Reset pseudo-random seed

if strcmp(IMU1_DATA, 'ON')      % If simulation of IMU1 data is required ...
    
    fprintf('Generating IMU1 ACCR data... \n')
    
    fb = acc_gen (ref, imu1); % Generate acc in the body frame
    imu1.fb = fb;
    
    fprintf('Generating IMU1 GYRO data... \n')
    
    wb = gyro_gen (ref, imu1);% Generate gyro in the body frame
    imu1.wb = wb;
    
    save imu1.mat imu1
    clear wb fb;
    
else
    fprintf('Loading IMU1 data... \n')
    
    load imu1.mat
end

%% SIMULATE IMU2

rng('shuffle')					% Reset pseudo-random seed

if strcmp(IMU2_DATA, 'ON')      % If simulation of IMU2 data is required ...
    
    fprintf('Generating IMU2 ACCR data... \n')
    
    fb = acc_gen (ref, imu2); % Generate acc in the body frame
    imu2.fb = fb;
    
    fprintf('Generating IMU2 GYRO data... \n')
    
    wb = gyro_gen (ref, imu2);% Generate gyro in the body frame
    imu2.wb = wb;
    
    save imu2.mat imu2
    
    clear wb fb;
    
else
    fprintf('Loading IMU2 data... \n')
    
    load imu2.mat
end

%% IMU1/GPS INTEGRATION WITH EFK

if strcmp(IMU1_INS, 'ON')
    
    fprintf('INS/GPS integration for IMU1... \n')
    
    % Sincronize GPS data with IMU data.
    
    % Guarantee that gps.t(1) < imu1.t(1) < gps.t(2)
    if (imu1.t(1) < gps.t(1)),
        
        igx  = find(imu1.t > gps.t(1), 1, 'first' );
        
        imu1.t  = imu1.t  (igx:end, :);
        imu1.fb = imu1.fb (igx:end, :);
        imu1.wb = imu1.wb (igx:end, :);        
    end
    
    % Guarantee that imu1.t(end-1) < gps.t(end) < imu1.t(end)
    if (imu1.t(end) <= gps.t(end)),
        
        fgx  = find(gps.t < imu1.t(end), 1, 'last' );
        
        gps.t   = gps.t  (1:fgx, :);
        gps.lat = gps.lat(1:fgx, :);
        gps.lon = gps.lon(1:fgx, :);
        gps.h   = gps.h  (1:fgx, :);
        gps.vel = gps.vel(1:fgx, :);
    end
    
    % Execute INS/GPS integration
    % ---------------------------------------------------------------------
    [imu1_e] = ins(imu1, gps, ref, 'quaternion', 'double');
    % ---------------------------------------------------------------------
    
    save imu1_e.mat imu1_e
    
else
    
    fprintf('Loading INS/GPS integration for IMU1... \n')
    
    load imu1_e.mat
end

%% IMU2/GPS INTEGRATION WITH EFK

if strcmp(IMU2_INS, 'ON')
    
    fprintf('\nINS/GPS integration for IMU2... \n')
    
    % Sincronize GPS data and IMU data.
    
    % Guarantee that gps.t(1) < imu2.t(1) < gps.t(2)
    if (imu2.t(1) < gps.t(1)),
        
        igx  = find(imu2.t > gps.t(1), 1, 'first' );
        
        imu2.t  = imu2.t  (igx:end, :);
        imu2.fb = imu2.fb (igx:end, :);
        imu2.wb = imu2.wb (igx:end, :);        
    end
    
    % Guarantee that imu2.t(end-1) < gps.t(end) < imu2.t(end)
    if (imu2.t(end) <= gps.t(end)),
        
        fgx  = find(gps.t < imu2.t(end), 1, 'last' );
        
        gps.t   = gps.t  (1:fgx, :);
        gps.lat = gps.lat(1:fgx, :);
        gps.lon = gps.lon(1:fgx, :);
        gps.h   = gps.h  (1:fgx, :);
        gps.vel = gps.vel(1:fgx, :);
    end
    
    % Execute INS/GPS integration
    % ---------------------------------------------------------------------
    [imu2_e] = ins(imu2, gps, ref, 'quaternion', 'double');
    % ---------------------------------------------------------------------
    
    save imu2_e.mat imu2_e
    
else
    
    fprintf('Loading INS/GPS integration for IMU2... \n')
    
    load imu2_e.mat
end

%% Print navigation time

to = (ref.t(end) - ref.t(1));

fprintf('\n>> Navigation time: %4.2f minutes or %4.2f seconds. \n', (to/60), to)

%% Print RMSE from IMU1

ref_1 = print_rmse (imu1_e, gps, ref, 'IMU1/GPS');

%% Print RMSE from IMU2

ref_2 = print_rmse (imu2_e, gps, ref, 'IMU2/GPS');

%% PLOT

if (strcmp(PLOT,'ON'))
    
    sig3_rr = abs(imu1_e.P_diag.^(0.5)).*3;
    
    % TRAJECTORY
    figure;
    plot3(ref.lon.*R2D, ref.lat.*R2D, ref.h)
    hold on
    plot3(ref.lon(1).*R2D, ref.lat(1).*R2D, ref.h(1), 'or', 'MarkerSize', 10, 'LineWidth', 2)
    axis tight
    title('TRAJECTORY')
    xlabel('Longitude [deg.]')
    ylabel('Latitude [deg.]')
    zlabel('Altitude [m]')
    grid
    
    % ATTITUDE
    figure;
    subplot(311)
    plot(ref_1.t, R2D.*ref_1.roll, '--k', imu1_e.t, R2D.*imu1_e.roll,'-b', imu2_e.t, R2D.*imu2_e.roll,'-r');
    ylabel('[deg]')
    xlabel('Time [s]')
    legend('REF', 'IMU1', 'IMU2');
    title('ROLL');
    
    subplot(312)
    plot(ref_1.t, R2D.*ref_1.pitch, '--k', imu1_e.t, R2D.*imu1_e.pitch,'-b', imu2_e.t, R2D.*imu2_e.pitch,'-r');
    ylabel('[deg]')
    xlabel('Time [s]')
    legend('REF', 'IMU1', 'IMU2');
    title('PITCH');
    
    subplot(313)
    plot(ref_1.t, R2D.* ref_1.yaw, '--k', imu1_e.t, R2D.*imu1_e.yaw,'-b', imu2_e.t, R2D.*imu2_e.yaw,'-r');
    ylabel('[deg]')
    xlabel('Time [s]')
    legend('REF', 'IMU1', 'IMU2');
    title('YAW');
    
    % ATTITUDE ERRORS
    figure;
    subplot(311)
    plot(imu1_e.t, (imu1_e.roll-ref_1.roll).*R2D, '-b', imu2_e.t, (imu2_e.roll-ref_2.roll).*R2D, '-r');
    hold on
    plot (gps.t, R2D.*sig3_rr(:,1), '--k', gps.t, -R2D.*sig3_rr(:,1), '--k' )
    ylabel('[deg]')
    xlabel('Time [s]')
    legend('IMU1', 'IMU2', '3\sigma');
    title('ROLL ERROR');
    
    subplot(312)
    plot(imu1_e.t, (imu1_e.pitch-ref_1.pitch).*R2D, '-b', imu2_e.t, (imu2_e.pitch-ref_2.pitch).*R2D, '-r');
    hold on
    plot (gps.t, R2D.*sig3_rr(:,2), '--k', gps.t, -R2D.*sig3_rr(:,2), '--k' )
    ylabel('[deg]')
    xlabel('Time [s]')
    legend('IMU1', 'IMU2', '3\sigma');
    title('PITCH ERROR');
    
    subplot(313)
    plot(imu1_e.t, (imu1_e.yaw-ref_1.yaw).*R2D, '-b', imu2_e.t, (imu2_e.yaw-ref_2.yaw).*R2D, '-r');
    hold on
    plot (gps.t, R2D.*sig3_rr(:,3), '--k', gps.t, -R2D.*sig3_rr(:,3), '--k' )
    ylabel('[deg]')
    xlabel('Time [s]')
    legend('IMU1', 'IMU2', '3\sigma');
    title('YAW ERROR');
    
    % VELOCITIES
    figure;
    subplot(311)
    plot(ref.t, ref.vel(:,1), '--k', gps.t, gps.vel(:,1),'-c', imu1_e.t, imu1_e.vel(:,1),'-b', imu2_e.t, imu2_e.vel(:,1),'-r');
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('REF', 'GPS', 'IMU1', 'IMU2');
    title('NORTH VELOCITY');
    
    subplot(312)
    plot(ref.t, ref.vel(:,2), '--k', gps.t, gps.vel(:,2),'-c', imu1_e.t, imu1_e.vel(:,2),'-b', imu2_e.t, imu2_e.vel(:,2),'-r');
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('REF', 'GPS', 'IMU1', 'IMU2');
    title('EAST VELOCITY');
    
    subplot(313)
    plot(ref.t, ref.vel(:,3), '--k', gps.t, gps.vel(:,3),'-c', imu1_e.t, imu1_e.vel(:,3),'-b', imu2_e.t, imu2_e.vel(:,3),'-r');
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('REF', 'GPS', 'IMU1', 'IMU2');
    title('DOWN VELOCITY');
    
    % VELOCITIES ERRORS
    figure;
    subplot(311)
    plot(gps.t, (gps.vel(:,1)-ref_g.vel(:,1)), '-c');
    hold on
    plot(imu1_e.t, (imu1_e.vel(:,1)-ref_1.vel(:,1)), '-b', imu2_e.t, (imu2_e.vel(:,1)-ref_2.vel(:,1)), '-r');
    hold on
    plot (gps.t, sig3_rr(:,4), '--k', gps.t, -sig3_rr(:,4), '--k' )
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('GPS', 'IMU1', 'IMU2', '3\sigma');
    title('VELOCITY NORTH ERROR');
    
    subplot(312)
    plot(gps.t, (gps.vel(:,2)-ref_g.vel(:,2)), '-c');
    hold on
    plot(imu1_e.t, (imu1_e.vel(:,2)-ref_1.vel(:,2)), '-b', imu2_e.t, (imu2_e.vel(:,2)-ref_2.vel(:,2)), '-r');
    hold on
    plot (gps.t, sig3_rr(:,5), '--k', gps.t, -sig3_rr(:,5), '--k' )
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('GPS', 'IMU1', 'IMU2', '3\sigma');
    title('VELOCITY EAST ERROR');
    
    subplot(313)
    plot(gps.t, (gps.vel(:,3)-ref_g.vel(:,3)), '-c');
    hold on
    plot(imu1_e.t, (imu1_e.vel(:,3)-ref_1.vel(:,3)), '-b', imu2_e.t, (imu2_e.vel(:,3)-ref_2.vel(:,3)), '-r');
    hold on
    plot (gps.t, sig3_rr(:,6), '--k', gps.t, -sig3_rr(:,6), '--k' )
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('GPS', 'IMU1', 'IMU2', '3\sigma');
    title('VELOCITY DOWN ERROR');
    
    % POSITION
    figure;
    subplot(311)
    plot(ref.t, ref.lat .*R2D, '--k', gps.t, gps.lat.*R2D, '-c', imu1_e.t, imu1_e.lat.*R2D, '-b', imu2_e.t, imu2_e.lat.*R2D, '-r');
    xlabel('Time [s]')
    ylabel('[deg]')
    legend('REF', 'GPS', 'IMU1', 'IMU2');
    title('LATITUDE');
    
    subplot(312)
    plot(ref.t, ref.lon .*R2D, '--k', gps.t, gps.lon.*R2D, '-c', imu1_e.t, imu1_e.lon.*R2D, '-b', imu2_e.t, imu2_e.lon.*R2D, '-r');
    xlabel('Time [s]')
    ylabel('[deg]')
    legend('REF', 'GPS', 'IMU1', 'IMU2');
    title('LONGITUDE');
    
    subplot(313)
    plot(ref.t, ref.h, '--k', gps.t, gps.h, '-c', imu1_e.t, imu1_e.h, '-b', imu2_e.t, imu2_e.h, '-r');
    xlabel('Time [s]')
    ylabel('[m]')
    legend('REF', 'GPS', 'IMU1', 'IMU2');
    title('ALTITUDE');
    
    % POSITION ERRORS
    % fh = @radicurv;
    % [RNs,REs] = arrayfun(fh, lat_rs);
    
    [RN,RE]  = radius(imu1_e.lat, 'double');
    LAT2M = RN + imu1_e.h;
    LON2M = (RE + imu1_e.h).*cos(imu1_e.lat);
    
    [RN,RE]  = radius(gps.lat, 'double');
    lat2m_g = RN + gps.h;
    lon2m_g = (RE + gps.h).*cos(gps.lat);
    
    figure;
    subplot(311)
    plot(gps.t, lat2m_g.*(gps.lat - ref_g.lat), '-c')
    hold on
    plot(imu1_e.t, LAT2M.*(imu1_e.lat - ref_1.lat), '-b')
    hold on
    plot(imu2_e.t, LAT2M.*(imu2_e.lat - ref_2.lat), '-r')
    hold on
    plot (gps.t, lat2m_g.*sig3_rr(:,7), '--k', gps.t, -lat2m_g.*sig3_rr(:,7), '--k' )
    xlabel('Time [s]')
    ylabel('[m]')
    legend('GPS', 'IMU1', 'IMU2', '3\sigma');
    title('LATITUDE ERROR');
    
    subplot(312)
    plot(gps.t, lon2m_g.*(gps.lon - ref_g.lon), '-c')
    hold on
    plot(imu1_e.t, LON2M.*(imu1_e.lon - ref_1.lon), '-b')
    hold on
    plot(imu2_e.t, LON2M.*(imu2_e.lon - ref_2.lon), '-r')
    hold on
    plot(gps.t, lon2m_g.*sig3_rr(:,8), '--k', gps.t, -lon2m_g.*sig3_rr(:,8), '--k' )
    xlabel('Time [s]')
    ylabel('[m]')
    legend('GPS', 'IMU1', 'IMU2', '3\sigma');
    title('LONGITUDE ERROR');
    
    subplot(313)
    plot(gps.t, (gps.h - ref_g.h), '-c')
    hold on
    plot(imu1_e.t, (imu1_e.h - ref_1.h), '-b')
    hold on
    plot(imu2_e.t, (imu2_e.h - ref_2.h), '-r')
    hold on
    plot(gps.t, sig3_rr(:,9), '--k', gps.t, -sig3_rr(:,9), '--k' )
    xlabel('Time [s]')
    ylabel('[m]')
    legend('GPS', 'IMU1', 'IMU2', '3\sigma');
    title('ALTITUDE ERROR');
    
end
