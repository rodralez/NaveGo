% Example of use of NAVEGO.
% Comparison between ADIS16405 IMU and ADIS16488 IMU performances
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
% Reference:
%           R. Gonzalez, J. Giribet, and H. Patiño. NaveGo: a
% simulation framework for low-cost integrated navigation systems,
% Journal of Control Engineering and Applied Informatics, vol. 17,
% issue 2, pp. 110-120, 2015. Eq. 26.
%
% Version: 001
% Date:    2014/09/18
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego
%
% KNOWN ISSUES: acceleration in Z axis is considered negative downward because
% Navego works (surprisingly) better this way. This Z axis orientation does
% not correspond with NED coordinates.

clc
close all
clear
matlabrc

fprintf('\nStarting simulation ... \n')

%% Global variables

global d2r

%% PARAMETERS

GPS_DATA  = 'ON';
IMU1_DATA = 'ON';
IMU2_DATA = 'ON';

IMU1_INS  = 'ON';
IMU2_INS  = 'ON';

RMSE      = 'ON';
PLOT      = 'ON';

if (~exist('GPS_DATA','var')),  GPS_DATA  = 'OFF'; end
if (~exist('IMU1_DATA','var')), IMU1_DATA = 'OFF'; end
if (~exist('IMU2_DATA','var')), IMU2_DATA = 'OFF'; end
if (~exist('IMU1_INS','var')),  IMU1_INS = 'OFF'; end
if (~exist('IMU2_INS','var')),  IMU2_INS = 'OFF'; end
if (~exist('RMSE','var')),      RMSE = 'OFF'; end
if (~exist('PLOT','var')),      PLOT = 'OFF'; end

%% CONVERSION CONSTANTS

ms2kmh = 3.6;       % m/s to km/h
d2r = (pi/180);     % degrees to radians
r2d = (180/pi);     % radians to degrees
mss2g = (1/9.81);   % m/s^2 to g
g2mss = 9.81;
kt2ms = 0.514444444;% knot to m/s

%% LOAD REF DATA

fprintf('Loading trajectory generator data... \n')

load ref.mat

%% IMU ADIS16405 error profile

ADIS16405.arw       = 2   .* ones(1,3);     % deg/root-hour
ADIS16405.vrw       = 0.2 .* ones(1,3);     % m/s/root-hour
ADIS16405.m_psd     = 0.066 .* ones(1,3);   % mgauss/root-Hz
ADIS16405.gb_fix    = 3 .* ones(1,3);       % deg/s
ADIS16405.ab_fix    = 50 .* ones(1,3);      % mg
ADIS16405.gb_drift  = 0.007 .* ones(1,3);   % deg/s
ADIS16405.ab_drift  = 0.2 .* ones(1,3);     % mg
ADIS16405.gcorr     = 100 .* ones(1,3);     % s
ADIS16405.acorr     = 100 .* ones(1,3);     % s
ADIS16405.freq      = 100;                  % Hz

% ref_1 = downsampling (ref, 1/ADIS16405.freq); % Resample ref if ref and imu
                                                % have differente operation frequencies.
ref_1 = ref;

dt = mean(diff(ref_1.t));                   % Mean period

imu1 = imu_err_profile(ADIS16405, dt);      % Transform IMU manufacturer units to SI units


%% IMU ADIS16488 error profile

ADIS16488.arw = 0.3     .* ones(1,3);       % degrees/root-hour
ADIS16488.vrw = 0.029   .* ones(1,3);       % m/s/root-hour
ADIS16488.m_psd = 0.054 .* ones(1,3);       % mgauss/root-Hz
ADIS16488.gb_fix = 0.2  .* ones(1,3);       % deg/s
ADIS16488.ab_fix = 16   .* ones(1,3);       % mg
ADIS16488.gb_drift = 6.5/3600  .* ones(1,3);% deg/s
ADIS16488.ab_drift = 0.1  .* ones(1,3);     % mg
ADIS16488.gcorr = 100 .* ones(1,3);         % s
ADIS16488.acorr = 100 .* ones(1,3);         % s
ADIS16488.freq = 100;                       % Hz

% imu2 = downsampling (ref, 1/ADIS16488.freq);  % Resample if ref and imu have differente operation frequencies.
                                                
ref_2 = ref;

dt = mean(diff(ref_2.t));                     % Mean period

imu2 = imu_err_profile(ADIS16488, dt);      % Transform IMU manufacturer error units to SI units.

%% GPS Garmin 5-18 Hz error profile

gps.stdm = [5, 5, 10];                 % m
gps.stdv = 0.1 * kt2ms .* ones(1,3);   % knot -> m/s
gps.larm = zeros(3,1);                 % Lever arm
gps.freq = 5;                          % Hz

%% SIMULATE GPS

rng('shuffle')                          % Reset pseudo-random seed

if strcmp(GPS_DATA, 'ON')               % If GPS simulated data is required ...
    
    fprintf('Simulating GPS data... \n')
    
    gps = gps_err_profile(ref.lat(1), ref.h(1), gps); % Transform GPS manufacturer error units to SI units.
    
    [gps, ref_g] = gps_gen(ref, gps);   % Generate GPS dataset from reference dataset.
                                        % ref_g is the ref dataset
                                        % resampled using GPS time.
    save gps.mat gps
    save ref_g.mat ref_g
    
else
    
    fprintf('Loading GPS data... \n')
    
    load gps.mat
    load ref_g.mat
end

%% SIMULATE imu1

rng('shuffle')                  % Reset pseudo-random seed

if strcmp(IMU1_DATA, 'ON')
    
    fprintf('Generating IMU1 ACCR data... \n')
    
    % KNOWN ISSUES: acceleration in Z axis is considered negative downward because
    % Navego works (surprisingly) better this way. This Z axis orientation does
    % not correspond with NED coordinates.
    
    fb = acc_gen (ref_1, imu1);
    imu1.fb = fb;
    
    fprintf('Generating IMU1 GYRO data... \n')
    
    wb = gyro_gen (ref_1, imu1);
    imu1.wb = wb;
    
    imu1.t = ref_1.t;
    imu1.freq = ref_1.freq;
    
    save imu1.mat imu1
    save ref_1.mat ref_1
    
    clear wb fb;
    
else
    fprintf('Loading IMU1 data... \n')
    
    load imu1.mat
    load ref_1.mat
end

%% SIMULATE imu2

rng('shuffle')

if strcmp(IMU2_DATA, 'ON')
    
    fprintf('Generating IMU2 ACCR data... \n')
    
    fb = acc_gen (ref_2, imu2);
    imu2.fb = fb;
    
    fprintf('Generating IMU2 GYRO data... \n')
    
    wb = gyro_gen (ref_2, imu2);
    imu2.wb = wb;
    
    imu2.t = ref_2.t;
    imu2.freq = ref_2.freq;
    
    save imu2.mat imu2
    save ref_2.mat ref_2
    
    clear wb fb;
    
else
    fprintf('Loading IMU2 data... \n')
    
    load imu2.mat
    load ref_2.mat
end

%% imu1/GPS INTEGRATION WITH FK

if strcmp(IMU1_INS, 'ON')
    
    fprintf('SINS/GPS integration using IMU1... \n')
    
    % Sincronize GPS data with IMU data.
    % Guarantee that gps.t(1) < imu1.t(1) < gps.t(2)
    if (imu1.t(1) < gps.t(1)),
        
        igx  = find(imu1.t > gps.t(1), 1, 'first' );
        
        imu1.t  = imu1.t  (igx:end, :);
        imu1.fb = imu1.fb (igx:end, :);
        imu1.wb = imu1.wb (igx:end, :);
        
        ref_1.t     = ref_1.t    (igx:end, :);
        ref_1.roll  = ref_1.roll (igx:end, :);
        ref_1.pitch = ref_1.pitch(igx:end, :);
        ref_1.yaw   = ref_1.yaw  (igx:end, :);
        ref_1.lat   = ref_1.lat  (igx:end, :);
        ref_1.lon   = ref_1.lon  (igx:end, :);
        ref_1.h     = ref_1.h    (igx:end, :);
        ref_1.vel   = ref_1.vel  (igx:end, :);
    end
    
    % Guarantee that imu1.t(end-1) < gps.t(end) < imu1.t(end)
    if (imu1.t(end) < gps.t(end)),
        
        fgx  = find(gps.t < imu1.t(end), 1, 'last' );
        
        gps.t   = gps.t  (1:fgx, :);
        gps.lat = gps.lat(1:fgx, :);
        gps.lon = gps.lon(1:fgx, :);
        gps.h   = gps.h  (1:fgx, :);
        gps.vel = gps.vel(1:fgx, :);
        ref_g.t   = ref_g.t  (1:fgx, :);
        ref_g.lat = ref_g.lat(1:fgx, :);
        ref_g.lon = ref_g.lon(1:fgx, :);
        ref_g.h   = ref_g.h  (1:fgx, :);
        ref_g.vel = ref_g.vel(1:fgx, :);
    else
        % Delete extra inertial meausurements begining at gps.t(end)
        fgx  = find(imu1.t > gps.t(end), 1, 'first' );
        
        imu1.t  = imu1.t  (1:fgx, :);
        imu1.fb = imu1.fb (1:fgx, :);
        imu1.wb = imu1.wb (1:fgx, :);
        
        ref_1.t     = ref_1.t    (1:fgx, :);
        ref_1.roll  = ref_1.roll (1:fgx, :);
        ref_1.pitch = ref_1.pitch(1:fgx, :);
        ref_1.yaw   = ref_1.yaw  (1:fgx, :);
        ref_1.lat   = ref_1.lat  (1:fgx, :);
        ref_1.lon   = ref_1.lon  (1:fgx, :);
        ref_1.h     = ref_1.h    (1:fgx, :);
        ref_1.vel   = ref_1.vel  (1:fgx, :);
    end
    
    % Execute INS/GPS integration
    % ---------------------------------------------------------------------
    [imu1_e] = ins(imu1, gps, ref_1, 'double');
    % ---------------------------------------------------------------------
    
    save imu1_e.mat imu1_e
    
else
    
    fprintf('Loading SINS/GPS integration using IMU1... \n')
    
    load imu1_e.mat
end

%% imu2/GPS INTEGRATION WITH FK

if strcmp(IMU2_INS, 'ON')
    
    fprintf('\nSINS/GPS integration using IMU2... \n')
    
    % Sincronize GPS data with IMU data.
    % Guarantee that gps.t(1) < imu2.t(1) < gps.t(2)
    if (imu2.t(1) < gps.t(1)),
        
        igx  = find(imu2.t > gps.t(1), 1, 'first' );
        
        imu2.t  = imu2.t  (igx:end, :);
        imu2.fb = imu2.fb (igx:end, :);
        imu2.wb = imu2.wb (igx:end, :);
        
        ref_2.t     = ref_2.t    (igx:end, :);
        ref_2.roll  = ref_2.roll (igx:end, :);
        ref_2.pitch = ref_2.pitch(igx:end, :);
        ref_2.yaw   = ref_2.yaw  (igx:end, :);
        ref_2.lat   = ref_2.lat  (igx:end, :);
        ref_2.lon   = ref_2.lon  (igx:end, :);
        ref_2.h     = ref_2.h    (igx:end, :);
        ref_2.vel   = ref_2.vel  (igx:end, :);
    end
    
    % Guarantee that imu2.t(end-1) < gps.t(end) < imu2.t(end)
    if (imu2.t(end) < gps.t(end)),
        
        fgx  = find(gps.t < imu2.t(end), 1, 'last' );
        
        gps.t = gps.t(1:fgx, :);
        gps.lat = gps.lat(1:fgx, :);
        gps.lon = gps.lon(1:fgx, :);
        gps.h   = gps.h(1:fgx, :);
        gps.vel = gps.vel(1:fgx, :);
        ref_g.t   = ref_g.t(1:fgx, :);
        ref_g.lat = ref_g.lat(1:fgx, :);
        ref_g.lon = ref_g.lon(1:fgx, :);
        ref_g.h   = ref_g.h(1:fgx, :);
        ref_g.vel = ref_g.vel(1:fgx, :);
    else
        % Eliminate extra inertial meausurements begining at gps.t(end)
        fgx  = find(imu2.t > gps.t(end), 1, 'first' );
        
        imu2.t  = imu2.t  (1:fgx, :);
        imu2.fb = imu2.fb (1:fgx, :);
        imu2.wb = imu2.wb (1:fgx, :);
        
        ref_2.t     = ref_2.t    (1:fgx, :);
        ref_2.roll  = ref_2.roll (1:fgx, :);
        ref_2.pitch = ref_2.pitch(1:fgx, :);
        ref_2.yaw   = ref_2.yaw  (1:fgx, :);
        ref_2.lat   = ref_2.lat  (1:fgx, :);
        ref_2.lon   = ref_2.lon  (1:fgx, :);
        ref_2.h     = ref_2.h    (1:fgx, :);
        ref_2.vel   = ref_2.vel  (1:fgx, :);
    end
    
    % Execute INS/GPS integration
    % ---------------------------------------------------------------------
    [imu2_e] = ins(imu2, gps, ref_2, 'double');
    % ---------------------------------------------------------------------
    
    save imu2_e.mat imu2_e
    
else
    
    fprintf('Loading SINS/GPS integration using IMU2... \n')
    
    load imu2_e.mat
end

%% Print navigation time

to = (ref.t(end) - ref.t(1));

fprintf('\n>> Navigation time: %4.3f min. or %4.3f sec. \n', (to/60), to)

%% Print RMSE IMU1

fe = max(size(imu1_e.t));
fr = max(size(ref_1.t));

% Adjust ref size if it is bigger than estimates
if (fe < fr)
    
    ref_1.t     = ref_1.t(1:fe, :);
    ref_1.roll  = ref_1.roll(1:fe, :);
    ref_1.pitch = ref_1.pitch(1:fe, :);
    ref_1.yaw   = ref_1.yaw(1:fe, :);
    ref_1.vel   = ref_1.vel(1:fe, :);
    ref_1.lat   = ref_1.lat(1:fe, :);
    ref_1.lon   = ref_1.lon(1:fe, :);
    ref_1.h     = ref_1.h(1:fe, :);
    ref_1.DCMnb = ref_1.DCMnb(1:fe, :);
end

[RN,RE] = radius(imu1_e.lat(1), 'double');
lat2m = (RN + double(imu1_e.h(1)));
lon2m = (RE + double(imu1_e.h(1))) .* cos(imu1_e.lat(1));

RMSE_roll   = rmse (imu1_e.roll ,  ref_1.roll)  .*r2d;
RMSE_pitch  = rmse (imu1_e.pitch,  ref_1.pitch) .*r2d;
% RMSE_yaw    = rmse (imu1_e.yaw,   ref_1.yaw).*r2d;

% Only compare those estimates that have a diff. < pi with respect to ref
idx = find ( abs(imu1_e.yaw - ref_1.yaw) < pi );
RMSE_yaw    = rmse (imu1_e.yaw(idx),   ref_1.yaw(idx)).*r2d;

RMSE_lat    = rmse (imu1_e.lat, ref_1.lat) .*lat2m;
RMSE_lon    = rmse (imu1_e.lon, ref_1.lon) .*lon2m;
RMSE_h      = rmse (imu1_e.h,         ref_1.h);
RMSE_vn     = rmse (imu1_e.vel(:,1),  ref_1.vel(:,1));
RMSE_ve     = rmse (imu1_e.vel(:,2),  ref_1.vel(:,2));
RMSE_vd     = rmse (imu1_e.vel(:,3),  ref_1.vel(:,3));

[RN,RE] = radius(gps.lat(1), 'double');
lat2m = (RN + double(gps.h(1)));
lon2m = (RE + double(gps.h(1))) .* cos(gps.lat(1));

RMSE_lat_g  = rmse (gps.lat, ref_g.lat) .*lat2m;
RMSE_lon_g  = rmse (gps.lon, ref_g.lon) .*lon2m;
RMSE_h_g    = rmse (gps.h-gps.larm(3), ref_g.h);
RMSE_vn_g   = rmse (gps.vel(:,1),   ref_g.vel(:,1));
RMSE_ve_g   = rmse (gps.vel(:,2),   ref_g.vel(:,2));
RMSE_vd_g   = rmse (gps.vel(:,3),   ref_g.vel(:,3));

% Print RMSE
fprintf( '\n>> RMSE IMU1\n');

fprintf( ' Roll,  IMU1 = %.4e deg.\n', ...
    RMSE_roll);
fprintf( ' Pitch, IMU1 = %.4e deg.\n', ...
    RMSE_pitch);
fprintf( ' Yaw,   IMU1 = %.4e deg.\n\n', ...
    RMSE_yaw);

fprintf( ' Vel. N, IMU1 = %.4e m/s, GPS = %.4e. m/s\n', ...
    RMSE_vn, RMSE_vn_g);
fprintf( ' Vel. E, IMU1 = %.4e m/s, GPS = %.4e. m/s\n', ...
    RMSE_ve, RMSE_ve_g);
fprintf( ' Vel. D, IMU1 = %.4e m/s, GPS = %.4e. m/s\n\n', ...
    RMSE_vd, RMSE_vd_g);

fprintf( ' Latitude,  IMU1 = %.4e m, GPS = %.4e. m\n', ...
    RMSE_lat, RMSE_lat_g);
fprintf( ' Longitude, IMU1 = %.4e m, GPS = %.4e. m\n', ...
    RMSE_lon, RMSE_lon_g);
fprintf( ' Altitude,  IMU1 = %.4e m, GPS = %.4e. m\n', ...
    RMSE_h, RMSE_h_g);

%% Print RMSE IMU2

fe = max(size(imu2_e.t));
fr = max(size(ref_2.t));

% Adjust ref size if it is bigger than estimates
if (fe < fr)
    
    ref_2.t     = ref_2.t(1:fe, :);
    ref_2.roll  = ref_2.roll(1:fe, :);
    ref_2.pitch = ref_2.pitch(1:fe, :);
    ref_2.yaw = ref_2.yaw(1:fe, :);
    ref_2.vel = ref_2.vel(1:fe, :);
    ref_2.lat = ref_2.lat(1:fe, :);
    ref_2.lon = ref_2.lon(1:fe, :);
    ref_2.h = ref_2.h(1:fe, :);
    ref_2.DCMnb = ref_2.DCMnb(1:fe, :);
end

[RN,RE] = radius(imu2_e.lat(1), 'double');
lat2m = (RN + double(imu2_e.h(1)));
lon2m = (RE + double(imu2_e.h(1))) .* cos(imu2_e.lat(1));

RMSE_roll   = rmse (imu2_e.roll ,     ref_2.roll)  .*r2d;
RMSE_pitch  = rmse (imu2_e.pitch,     ref_2.pitch) .*r2d;
% RMSE_yaw    = rmse (imu1_e.yaw,   ref_1.yaw).*r2d;

% Only compare those estimates that have a diff. < pi with respect to ref
idx = find ( abs(imu2_e.yaw - ref_2.yaw) < pi );
RMSE_yaw    = rmse (imu2_e.yaw(idx),   ref_2.yaw(idx)).*r2d;

RMSE_lat    = rmse (imu2_e.lat, ref_2.lat) .*lat2m;
RMSE_lon    = rmse (imu2_e.lon, ref_2.lon) .*lon2m;
RMSE_h      = rmse (imu2_e.h,         ref_2.h);
RMSE_vn     = rmse (imu2_e.vel(:,1),  ref_2.vel(:,1));
RMSE_ve     = rmse (imu2_e.vel(:,2),  ref_2.vel(:,2));
RMSE_vd     = rmse (imu2_e.vel(:,3),  ref_2.vel(:,3));

[RN,RE] = radius(gps.lat(1), 'double');
lat2m = (RN + double(gps.h(1)));
lon2m = (RE + double(gps.h(1))) .* cos(gps.lat(1));

RMSE_lat_g  = rmse (gps.lat, ref_g.lat) .*lat2m;
RMSE_lon_g  = rmse (gps.lon, ref_g.lon) .*lon2m;
RMSE_h_g    = rmse (gps.h-gps.larm(3), ref_g.h); %
RMSE_vn_g   = rmse (gps.vel(:,1),   ref_g.vel(:,1));
RMSE_ve_g   = rmse (gps.vel(:,2),   ref_g.vel(:,2));
RMSE_vd_g   = rmse (gps.vel(:,3),   ref_g.vel(:,3));

% Print into console
fprintf( '\n>> RMSE IMU2\n');

fprintf( ' Roll,  IMU2 = %.4e deg.\n', ...
    RMSE_roll);
fprintf( ' Pitch, IMU2 = %.4e deg.\n', ...
    RMSE_pitch);
fprintf( ' Yaw,   IMU2 = %.4e deg.\n\n', ...
    RMSE_yaw);

fprintf( ' Vel. N, IMU2 = %.4e m/s, GPS = %.4e. m/s\n', ...
    RMSE_vn, RMSE_vn_g);
fprintf( ' Vel. E, IMU2 = %.4e m/s, GPS = %.4e. m/s\n', ...
    RMSE_ve, RMSE_ve_g);
fprintf( ' Vel. D, IMU2 = %.4e m/s, GPS = %.4e. m/s\n\n', ...
    RMSE_vd, RMSE_vd_g);

fprintf( ' Latitude,  IMU2 = %.4e m, GPS = %.4e. m\n', ...
    RMSE_lat, RMSE_lat_g);
fprintf( ' Longitude, IMU2 = %.4e m, GPS = %.4e. m\n', ...
    RMSE_lon, RMSE_lon_g);
fprintf( ' Altitude,  IMU2 = %.4e m, GPS = %.4e. m\n', ...
    RMSE_h, RMSE_h_g);

%% PLOT

if (strcmp(PLOT,'ON'))
    
    sig3_rr = abs(imu1_e.P_diag.^(0.5)).*3;
    
    % TRAJECTORY
    figure;
    plot3(ref.lon.*r2d, ref.lat.*r2d, ref.h)
    hold on
    plot3(ref.lon(1).*r2d, ref.lat(1).*r2d, ref.h(1), 'or', 'MarkerSize', 10, 'LineWidth', 2)
    axis tight
    title('TRAJECTORY')
    xlabel('Longitude [deg.]')
    ylabel('Latitude [deg.]')
    zlabel('Altitude [m]')
    grid
    
    % ATTITUDE
    figure;
    subplot(311)
    plot(ref_1.t, r2d.*ref_1.roll, '--k', imu1_e.t, r2d.*imu1_e.roll,'-b', imu2_e.t, r2d.*imu2_e.roll,'-r');
    ylabel('[deg]')
    xlabel('Time [s]')
    legend('REF', 'IMU1', 'IMU2');
    title('ROLL');
    
    subplot(312)
    plot(ref_1.t, r2d.*ref_1.pitch, '--k', imu1_e.t, r2d.*imu1_e.pitch,'-b', imu2_e.t, r2d.*imu2_e.pitch,'-r');
    ylabel('[deg]')
    xlabel('Time [s]')
    legend('REF', 'IMU1', 'IMU2');
    title('PITCH');
    
    subplot(313)
    plot(ref_1.t, r2d.* ref_1.yaw, '--k', imu1_e.t, r2d.*imu1_e.yaw,'-b', imu2_e.t, r2d.*imu2_e.yaw,'-r');
    ylabel('[deg]')
    xlabel('Time [s]')
    legend('REF', 'IMU1', 'IMU2');
    title('YAW');
    
    % ATTITUDE ERRORS
    figure;
    subplot(311)
    plot(imu1_e.t, (imu1_e.roll-ref_1.roll).*r2d, '-b', imu2_e.t, (imu2_e.roll-ref_2.roll).*r2d, '-r');
    hold on
    plot (gps.t, r2d.*sig3_rr(:,1), '--k', gps.t, -r2d.*sig3_rr(:,1), '--k' )
    ylabel('[deg]')
    xlabel('Time [s]')
    legend('IMU1', 'IMU2', '3\sigma');
    title('ROLL ERROR');
    
    subplot(312)
    plot(imu1_e.t, (imu1_e.pitch-ref_1.pitch).*r2d, '-b', imu2_e.t, (imu2_e.pitch-ref_2.pitch).*r2d, '-r');
    hold on
    plot (gps.t, r2d.*sig3_rr(:,2), '--k', gps.t, -r2d.*sig3_rr(:,2), '--k' )
    ylabel('[deg]')
    xlabel('Time [s]')
    legend('IMU1', 'IMU2', '3\sigma');
    title('PITCH ERROR');
    
    subplot(313)
    plot(imu1_e.t, (imu1_e.yaw-ref_1.yaw).*r2d, '-b', imu2_e.t, (imu2_e.yaw-ref_2.yaw).*r2d, '-r');
    hold on
    plot (gps.t, r2d.*sig3_rr(:,3), '--k', gps.t, -r2d.*sig3_rr(:,3), '--k' )
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
    plot(ref.t, ref.lat .*r2d, '--k', gps.t, gps.lat.*r2d, '-c', imu1_e.t, imu1_e.lat.*r2d, '-b', imu2_e.t, imu2_e.lat.*r2d, '-r');
    xlabel('Time [s]')
    ylabel('[deg]')
    legend('REF', 'GPS', 'IMU1', 'IMU2');
    title('LATITUDE');
    
    subplot(312)
    plot(ref.t, ref.lon .*r2d, '--k', gps.t, gps.lon.*r2d, '-c', imu1_e.t, imu1_e.lon.*r2d, '-b', imu2_e.t, imu2_e.lon.*r2d, '-r');
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
    lat2m = RN + imu1_e.h;
    lon2m = (RE + imu1_e.h).*cos(imu1_e.lat);
    
    [RN,RE]  = radius(gps.lat, 'double');
    lat2m_g = RN + gps.h;
    lon2m_g = (RE + gps.h).*cos(gps.lat);
    
    figure;
    subplot(311)
    plot(gps.t, lat2m_g.*(gps.lat - ref_g.lat), '-c')
    hold on
    plot(imu1_e.t, lat2m.*(imu1_e.lat - ref_1.lat), '-b')
    hold on
    plot(imu2_e.t, lat2m.*(imu2_e.lat - ref_2.lat), '-r')
    hold on
    plot (gps.t, lat2m_g.*sig3_rr(:,7), '--k', gps.t, -lat2m_g.*sig3_rr(:,7), '--k' )
    xlabel('Time [s]')
    ylabel('[m]')
    legend('GPS', 'IMU1', 'IMU2', '3\sigma');
    title('LATITUDE ERROR');
    
    subplot(312)
    plot(gps.t, lon2m_g.*(gps.lon - ref_g.lon), '-c')
    hold on
    plot(imu1_e.t, lon2m.*(imu1_e.lon - ref_1.lon), '-b')
    hold on
    plot(imu2_e.t, lon2m.*(imu2_e.lon - ref_2.lon), '-r')
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
