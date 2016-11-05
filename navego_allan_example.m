% navego_allan_example: example of how to implement the Allan variance 
% procedure with NaveGo functions.
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
%       Sensonor AS. STIM300 Inertial Measurement Unit 
%       http://www.sensonor.com/gyro-products/inertial-measurement-units/stim300.aspx7.
%
%       Four hours of static measurements from STIM300 IMU were generously
%       provided by Prof. Zhu, Dr. Yang, and Mr. Bo Sun, all from the Laboratory 
%       of Precision Measuring Technology and Instruments, Tianjin University, 
%       Tianjin, China.
%
% Version: 001
% Date:    2016/11/04
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

clc
clear
close all
matlabrc

versionstr = 'NaveGo, release v0.7.1-alpha';

fprintf('\n%s.\n', versionstr)
fprintf('\nNaveGo: starting Allan variance analysis ... \n')

load stim300

%% ALLAN VARIANCE FOR STIM300 IMU

[stim300, tau1_m, allan1_m] = allan_imu (stim300);

imu1_arw = stim300.arw
imu1_vrw = stim300.vrw

imu1_ab_drift = stim300.ab_drift
imu1_gb_drift = stim300.gb_drift

imu1_ab_corr = stim300.ab_corr
imu1_gb_corr = stim300.gb_corr
