% navego_example_sensor_read.m is an example of how to use NaveGo function
% sensor_read.m. This functions tries to be a standard method to extract 
% data from text files that contains a particular sensor's measurements. 
% Typically, this file will have .csv extension.
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
% Version: 001
% Date:    2021/05/05
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego 

clc
close all
clear
matlabrc

addpath ../../data-acquisition/

navego_print_version;

fprintf('navego_example_sensor_read: processing data from a Microstrain IMU... \n')

%% GET DATA FROM FILE

file_name = 'miscrostrain.csv';
head_lines = 16;
fields_line = 16;
specific_delimiter = ',';

ustrain_data = sensor_read(file_name, head_lines, fields_line, specific_delimiter);

%% TRANSFORM DATA TO NAVEGO FORMAT

G =  9.80665;       % Gravity constant, m/s^2

ustrain_imu.t  = ustrain_data.GPS_TOW; % seconds
ustrain_imu.fb = [ustrain_data.X_Accel__x8004_ , ustrain_data.Y_Accel__x8004_ , ustrain_data.Z_Accel__x8004_] * G; % m/s^2
ustrain_imu.wb = [ustrain_data.X_Gyro__x8005_ , ustrain_data.Y_Gyro__x8005_ , ustrain_data.Z_Gyro__x8005_]; % rad/s

ustrain_imu.lat = ustrain_data.Lat__x8103_ ; % rad
ustrain_imu.lon = ustrain_data.Long__x8103_ ; % rad 
ustrain_imu.h = ustrain_data.Height__x8103_ ; % m

save ustrain_imu ustrain_imu
