% navego_example_real_gnss_outage: post-processing integration of Ekinox
% IMU and Ekinox GNSS data. Two GNSS outages are forced.
%
% The main goal is to integrate MPU-6000 IMU and Ekinox-D GNSS measurements
% and test INS/GNSS systems performance under two GNSS outages.
%
% Sensors dataset was generated driving a car through the streets of
% Turin city (Italy).
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
%
%   SBG Systems. SBG Ekinox-D High Accuracy Inertial System Brochure,
% Tactical grade MEMS Inertial Systems, v1.0. February 2014.
%
%   R. Gonzalez and P. Dabove. Performance Assessment of an Ultra Low-Cost
% Inertial Measurement Unit for Ground Vehicle Navigation. Sensors 2019,
% 19(18). https://www.mdpi.com/530156.
%
% Version: 006
% Date:    2021/12/15
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

% NOTE: NaveGo supposes that IMU is aligned with respect to body-frame as
% X-forward, Y-right and Z-down.
%
% NOTE: NaveGo assumes that yaw angle (heading) is positive clockwise.

clc
close all
clear
matlabrc

addpath ../../ins/
addpath ../../ins-gnss/
addpath ../../conversions/
addpath ../../performance-analysis/
addpath ../../misc/
addpath ../../plot/
addpath ../../simulation/

navego_print_version;

fprintf('\nNaveGo: starting real INS/GNSS integration... \n')

%% PARAMETERS

% Comment any of the following parameters in order to NOT execute a
% particular portion of code

INS_GNSS = 'ON';
GNSS_OUTAGE = 'ON';
PLOT     = 'ON';

if (~exist('INS_GNSS','var')), INS_GNSS = 'OFF'; end
if (~exist('PLOT','var')),     PLOT     = 'OFF'; end
if (~exist('GNSS_OUTAGE','var')),      GNSS_OUTAGE = 'OFF'; end

%% CONVERSION CONSTANTS

G =  9.80665;       % Gravity constant, m/s^2
G2MSS = G;          % g to m/s^2
MSS2G = (1/G);      % m/s^2 to g

D2R = (pi/180);     % degrees to radians
R2D = (180/pi);     % radians to degrees

KT2MS = 0.514444;   % knot to m/s
MS2KMH = 3.6;       % m/s to km/h

%% REF DATA

% Reference dataset was obtained by processing Ekinox IMU and Ekinox GNSS
% with tighly-coupled integration by Inertial Explorer software package.

% Dataset from time 138000 (TOW) to 139255 (TOW).

fprintf('NaveGo: loading reference data... \n')

load ref

%% EKINOX IMU

% fprintf('NaveGo: loading Ekinox IMU data... \n')
%
% load ekinox_imu
%
% imu = ekinox_imu;

%% MPU-6000 IMU

fprintf('NaveGo: loading MPU-6000 IMU data... \n')

load mpu6000_imu

imu = mpu6000_imu;

%% EKINOX GNSS

fprintf('NaveGo: loading Ekinox GNSS data... \n')

load ekinox_gnss

gnss = ekinox_gnss;

gnss.eps = mean(diff(imu.t)) / 2; %  A rule of thumb for choosing eps.

%% GNSS OUTAGE

if (strcmp(GNSS_OUTAGE, 'ON'))
    
    % Force two GNSS outage paths
    
    % GNSS OUTAGE 1, TIME INTERVAL
    gout_sta_1 = 138906;          % (seconds)
    gout_end_1 = gout_sta_1 + 10;     % (seconds)
    
    % GNSS OUTAGE 2, TIME INTERVAL
    gout_sta_2 = 139170;          % (seconds)
    gout_end_2 = gout_sta_2 + 10;     % (seconds)
    
    times_out = [gout_sta_1, gout_end_1, ...
        gout_sta_2, gout_end_2];
    
    gnss = gnss_outage(gnss, times_out);
    
end

%% NAVIGATION TIME

to = (ref.t(end) - ref.t(1));

fprintf('NaveGo: navigation time under analysis is %.2f minutes or %.2f seconds. \n', (to/60), to)

%% INS/GNSS INTEGRATION

if strcmp(INS_GNSS, 'ON')
    
    fprintf('NaveGo: processing INS/GNSS integration... \n')
    
    % Execute INS/GNSS integration
    % ---------------------------------------------------------------------
    nav_outage = ins_gnss(imu, gnss, 'quaternion'); %
    % ---------------------------------------------------------------------
    
    save nav_outage nav_outage
    
else
    
    load nav_outage
end

%% TRAVELED DISTANCE

distance = gnss_distance (nav_outage.lat, nav_outage.lon);

fprintf('NaveGo: distance traveled by the vehicle is %.2f meters or %.2f km. \n', distance, distance/1000)

%% ANALYSIS OF PERFORMANCE FOR A CERTAIN PART OF THE INS/GNSS DATASET

% COMPLETE TEST
tmin = 138000;      % Entering PoliTo parking (seconds)
tmax = 139255;      % Entering tunnel (seconds)

% Sincronize REF data to tmin and tmax
idx  = find(ref.t > tmin, 1, 'first' );
fdx  = find(ref.t < tmax, 1, 'last' );
if(isempty(idx) || isempty(fdx))
    error('ref: empty index')
end

ref.t       = ref.t    (idx:fdx);
ref.roll    = ref.roll (idx:fdx);
ref.pitch   = ref.pitch(idx:fdx);
ref.yaw     = ref.yaw  (idx:fdx);
ref.lat     = ref.lat  (idx:fdx);
ref.lon     = ref.lon  (idx:fdx);
ref.h       = ref.h    (idx:fdx);
ref.vel     = ref.vel  (idx:fdx, :);

%% INTERPOLATION OF INS/GNSS DATASET

% INS/GNSS estimates and GNSS data are interpolated according to the
% reference dataset.

[nav_i,  ref_n] = navego_interpolation (nav_outage, ref);
[gnss_i, ref_g] = navego_interpolation (gnss,  ref);

% Force GNSS OUTAGE in GNSS interpotated data
if (strcmp(GNSS_OUTAGE, 'ON'))
    
    gnss_i = gnss_outage(gnss_i, times_out);
    ref_g  = gnss_outage(ref_g,  times_out);
end

%% NAVIGATION RMSE

rmse_v = print_rmse (nav_i, gnss_i, ref_n, ref_g, 'Ekinox IMU/GNSS');

%% RMSE TO CVS FILE

csvwrite('nav_ekinox_outage.csv', rmse_v);

%% NAVIGATION DATA TO CSV FILE

fprintf('\n');
navego_nav2csv(nav_outage);

%% PLOTS

if (strcmp(PLOT,'ON'))
    
    navego_plot_main (ref, gnss, nav_outage, gnss_i, nav_i, ref_g, ref_n, ...
        GNSS_OUTAGE, times_out );
end
