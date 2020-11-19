% navego_example_real_ekinox: post-processing integration of both Ekinox 
% IMU and Ekinox GNSS data.
%
% Main goal: to integrate IMU and GNSS measurements from Ekinox-D sensor 
% which includes both IMU and GNSS sensors.
%
% Sensors dataset was generated driving a vehicle through the streets of 
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
% Reference:
%
%   SBG Systems. SBG Ekinox-D High Accuracy Inertial System Brochure, 
% Tactical grade MEMS Inertial Systems, v1.0. February 2014. 
%
%   R. Gonzalez and P. Dabove. Performance Assessment of an Ultra Low-Cost 
% Inertial Measurement Unit for Ground Vehicle Navigation. Sensors 2019,  
% 19(18). https://www.mdpi.com/530156.
%
% Version: 003
% Date:    2020/11/19
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

% NOTE: NaveGo assumes that IMU is aligned with respect to body-frame as 
% X-forward, Y-right, and Z-down.

clc
close all
clear
matlabrc

addpath ../../.
addpath ../../simulation/
addpath ../../conversions/
addpath ../../performance_analysis/

versionstr = 'NaveGo, release v1.2';

fprintf('\n%s.\n', versionstr)
fprintf('\nNaveGo: starting real INS/GNSS integration... \n')

%% PARAMETERS

% Comment any of the following parameters in order to NOT execute a 
% particular portion of code

INS_GNSS = 'ON';
PLOT     = 'ON';

if (~exist('INS_GNSS','var')), INS_GNSS = 'OFF'; end
if (~exist('PLOT','var')),     PLOT     = 'OFF'; end

%% CONVERSION CONSTANTS

G =  9.80665;       % Gravity constant, m/s^2
G2MSS = G;          % g to m/s^2
MSS2G = (1/G);      % m/s^2 to g

D2R = (pi/180);     % degrees to radians
R2D = (180/pi);     % radians to degrees

KT2MS = 0.514444;   % knot to m/s
MS2KMH = 3.6;       % m/s to km/h

%% LOAD REF DATA

% Reference dataset was obtained by processing Ekinox IMU and Ekinox GNSS 
% with tighly-coupled integration by Inertial Explorer software package.

fprintf('NaveGo: loading reference data... \n')

load ref

%% EKINOX IMU 

fprintf('NaveGo: loading Ekinox IMU data... \n')

load ekinox_imu

%% EKINOX GNSS 

fprintf('NaveGo: loading Ekinox GNSS data... \n')

load ekinox_gnss

% ekinox_gnss.eps = mean(diff(ekinox_imu.t)) / 2; %  A rule of thumb for choosing eps.

%% Print navigation time

to = (ref.t(end) - ref.t(1));

fprintf('NaveGo: navigation time is %.2f minutes or %.2f seconds. \n', (to/60), to)

%% INS/GNSS integration

if strcmp(INS_GNSS, 'ON')
    
    fprintf('NaveGo: INS/GNSS integration... \n')
    
    % Execute INS/GPS integration
    % ---------------------------------------------------------------------
    nav_ekinox = ins_gnss(ekinox_imu, ekinox_gnss, 'quaternion'); %
    % ---------------------------------------------------------------------
    
    save nav_ekinox.mat nav_ekinox
    
else
    
    load nav_ekinox
end

%% Printing traveled distance

distance = gnss_distance (nav_ekinox.lat, nav_ekinox.lon);

fprintf('NaveGo: distance traveled by the vehicle is %.2f meters or %.2f km. \n', distance, distance/1000)

%% ANALYSIS OF PERFORMANCE FOR A CERTAIN PART OF THE INS/GNSS DATASET

tmin_rmse = ref.t(1); 
tmax_rmse = ref.t(end); 

% Sincronize REF data to tmin and tmax
idx  = find(ref.t > tmin_rmse, 1, 'first' );
fdx  = find(ref.t < tmax_rmse, 1, 'last' );
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

%% Interpolation of INS/GNSS dataset 

% INS/GNSS estimates and GNSS data are interpolated according to the
% reference dataset.

[nav_i,  ref_n] = navego_interpolation (nav_ekinox, ref);
[gnss_i, ref_g] = navego_interpolation (ekinox_gnss, ref);

%% Printing RMSE from INS/GNSS data

rmse_v = print_rmse (nav_i, gnss_i, ref_n, ref_g, 'Ekinox INS/GNSS');

%% Saving RMSE to CVS file

csvwrite('ekinox.csv', rmse_v);

%% PLOT

if (strcmp(PLOT,'ON'))
    
   navego_plot (ref, ekinox_gnss, nav_ekinox, gnss_i, nav_i, ref_g, ref_n)
end

%% Performance analysis of the Kalman filter

fprintf('\nNaveGo: Kalman filter performance analysis...\n') 

kf_analysis (nav_ekinox)

