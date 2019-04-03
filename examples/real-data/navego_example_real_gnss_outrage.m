% navego_example_real_gnss_outrage: Example of how to use NaveGo to 
% post-process both real IMU and GNSS data. 
%
% Main goal: to integrate IMU and GNSS measurements from Ekinox-D sensor 
% which includes both IMU and GNSS sensors. Two GNSS outrage periods are 
% forced.
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
% References
%
%   SBG Systems. SBG Ekinox-D High Accuracy Inertial System Brochure, 
% Tactical grade MEMS Inertial Systems, v1.0. February 2014. 
%
% Version: 001
% Date:    2019/01/15
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

% Dataset from time 138000 (TOW) to 139255 (TOW).

fprintf('NaveGo: loading reference data... \n')

load ref

%% EKINOX IMU 

fprintf('NaveGo: loading Ekinox IMU data... \n')

load imu

%% EKINOX GNSS 

fprintf('NaveGo: loading Ekinox GNSS data... \n')

load gnss

% Force a GNSS outrage of 1 minute from 138500 (TOW)

gdx =  find(gnss.t >= 138500, 1, 'first');
fdx = ceil (gdx + gnss.freq * 60);
outrage_t =  gnss.t(fdx) - gnss.t(gdx); 

fprintf('NaveGo: forced GNSS outrage of %.2f seconds, from time %.4f to %.4f... \n', outrage_t, gnss.t(gdx), gnss.t(fdx) )

% Delete elements from gdx to fdx

gnss.t (gdx:fdx) = []; 
gnss.lat (gdx:fdx) = [];
gnss.lon (gdx:fdx) = [];
gnss.h (gdx:fdx) = []; 
gnss.vel (gdx:fdx, :) = []; 

% Force a GNSS outrage of 2 minutes from 138900 (TOW)

gdx =  find(gnss.t >= 138900, 1, 'first');
fdx = ceil (gdx + gnss.freq * 60 * 2);
outrage_t =  gnss.t(fdx) - gnss.t(gdx); 

fprintf('NaveGo: forced GNSS outrage of %.2f seconds, from time %.4f to %.4f... \n', outrage_t, gnss.t(gdx), gnss.t(fdx) )

% Delete elements from gdx to fdx

gnss.t (gdx:fdx) = []; 
gnss.lat (gdx:fdx) = [];
gnss.lon (gdx:fdx) = [];
gnss.h (gdx:fdx) = []; 
gnss.vel (gdx:fdx, :) = [];

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

% Dataset from time 138000 (TOW) to 139255 (TOW).

% COMPLETE TRAJECTORY
tmin = 138000; % Entering PoliTo parking.
% tmax = 138380; % Short test. Leaving PoliTo parking.
tmax = 139255; % Before entering tunnel

% OUTRAGE 1
% tmin = 138500;
% tmax = 138500 + 60; % 1 minute outrage

% OUTRAGE 2
% tmin = 138900; 
% tmax = 138900 + 120; % 2 minutes outrage

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

%% Interpolate INS/GNSS dataset 

% INS/GNSS estimates and GNSS data are interpolated according to the
% reference dataset.

[nav_ref,  ref_n] = navego_interpolation (nav_e, ref);
[gnss_ref, ref_g] = navego_interpolation (gnss,  ref);

%% Print navigation time

to = (ref.t(end) - ref.t(1));

fprintf('NaveGo: navigation time under analysis is %.2f minutes or %.2f seconds. \n', (to/60), to)

%% Print RMSE from INS/GNSS data

rmse_v = print_rmse (nav_ref, gnss_ref, ref_n, ref_g, 'Ekinox IMU/GNSS');

%% Save RMSE to CVS file

csvwrite('ekinox.csv', rmse_v);

%% PLOT

if (strcmp(PLOT,'ON'))
    
   navego_plot (ref, gnss, nav_e, gnss_ref, nav_ref, ref_g, ref_n)
end
