% Example of how to use NaveGo to post-process real IMU and GNSS data.
%
% Main goal: to integrate IMU and GNSS measurements from Ekinox-D IMU.
% The dataset was generated driving a vehicle through the streets of Turin 
% city (Italy).
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
%    SBG Systems. SBG Ekinox-D High Accuracy Inertial System Brochure, 
% TACTICAL GRADE MEMS Inertial Systems, V1.0. February 2014. 
%
% Version: 001
% Date:    2018/10/16
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

% NOTE: NaveGo supposes that IMU is aligned with respect to body-frame as 
% X-forward, Y-right, and Z-down.

clc
close all
clear
matlabrc

addpath ../../
addpath ../../simulation/
addpath ../../conversions/

versionstr = 'NaveGo, release v1.1';

fprintf('\n%s.\n', versionstr)
fprintf('\nNaveGo: starting simulation ... \n')


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

% Reference dataset was obtained by processing IMU Ekinox and D-GNSS Ekinox
% with tighly-coupled integration by Inertial Explorer software.

fprintf('Loading reference data... \n')

load ref

%% IMU EKINOX error profile

fprintf('Loading Ekinox IMU data... \n')

load imu


%% GPS

fprintf('Loading Ekinox GNSS data... \n')

load gnss


%% Print navigation time

to = (ref.t(end) - ref.t(1));

fprintf('NaveGo: navigation time is %.2f minutes or %.2f seconds. \n', (to/60), to)

%% INS/GNSS integration

if strcmp(INS_GNSS, 'ON')
    
    fprintf('NaveGo: INS/GNSS integration... \n')
    
    % Execute INS/GPS integration
    % ---------------------------------------------------------------------
    nav_e = ins_gnss(imu, gnss, 'quaternion'); %
    % ---------------------------------------------------------------------
    
    save nav_e.mat nav_e
    
else
    
    load nav_e
end

%% ANALYZE A CERTAIN PART OF THE INS/GNSS DATASET

% SHORT TEST
tmin = 138000; % Entering PoliTo parking.
% tmax = 138380; % Leaving PoliTo parking.

% STREET STRETCH
% tmin = 138000; %
% tmax = 139190; %

% TUNNEL 1 STRETCH
% tmin = 139268.5; % entering tunnel
tmax = 139305.5; % leaving tunnel

% Sincronize REF data to tmin and tmax
idx  = find(ref.t > tmin, 1, 'first' );
fdx  = find(ref.t < tmax, 1, 'last' );
if(isempty(idx) || isempty(fdx))
    error('ref: empty index')
end

ref.t       = ref.t(idx:fdx);
ref.roll    = ref.roll(idx:fdx);
ref.pitch   = ref.pitch(idx:fdx);
ref.yaw     = ref.yaw(idx:fdx);
ref.lat     = ref.lat(idx:fdx);
ref.lon     = ref.lon(idx:fdx);
ref.h       = ref.h(idx:fdx);
ref.vel     = ref.vel(idx:fdx, :);

%% Interpolate INS/GNSS dataset 

% INS/GNSS estimates and GNSS data are interpolated according to the
% reference dataset.

[nav_ref,  ref_n] = navego_interpolation (nav_e, ref);
[gnss_ref, ref_g] = navego_interpolation (gnss,  ref);

%% Print navigation time

to = (ref.t(end) - ref.t(1));

fprintf('NaveGo: navigation time under analysis is %.2f minutes or %.2f seconds. \n', (to/60), to)

%% Print RMSE from INS/GNSS data

rmse_v = print_rmse (nav_ref, gnss_ref, ref_n, ref_g, 'Ekinox/Ekinox GNSS');

%% Save RMSE to CVS file

csvwrite('ekinox.csv', rmse_v);


%% PLOT

if (strcmp(PLOT,'ON'))
    
   navego_plot (ref, gnss, nav_e, gnss_ref, nav_ref, ref_g, ref_n)
end
