function h764 = h764_read(file)
% h764_read: reads .cvs files created by Microstrain software.
%
% INPUT:
%   fname, file name (string).
%
% OUTPUT:
%   h764, data structure.
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
%   Reference:
%       Microstrain. 3DM-GX3®-35 Data Communications Protocol. 8500-0014
%       Revision 010. Oct. 1 2013.
%
% Version: 001
% Date:    2018/09/12
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

%% CONSTANTS

G =   9.80665;
D2R = pi/180;
F2M = 0.3048; % feet to meters

%% OPEN FILE

h764_f = fopen(file, 'r');
if h764_f == -1
    error('h764_read: ERROR: %s file not found', file)
end

%% TOTAL NUMBER OF LINES

% lines = nnz(fread(h764_f) == 10);
% fprintf('microstrain_read: %s file has %d lines. \n', fname, lines);
% 
% % Set pointer back to the beginning
% fseek(h764_f,0,'bof');

%% HEADER

rows = 6;

h764_h = cell(rows,1);
for i =1:rows
    h764_h{i,1} = fgets(h764_f);
end

%% Total number of columns

% Copy and paste first data row
commas = strfind('37858,489598.602853189222515,0.00388539855706539552505,-0.00376263159286339011339,-0.129151674245668923735,5.3955554362259335212e-006,-5.39807461561521608465e-006,-8.60731162082906193771e-009,      0,           0,           0,           0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,', ',');
col = max(size(commas)) + 1;

% Create pattern for textscan()
str = [];
for i=1:col
    
    str = [str '%f '] ;
end

%% DATA

% "SYSTEM_TT","GPS_TIME","X_DEL_V","Y_DEL_V","Z_DEL_V","X_DEL_T","Y_DEL_T","Z_DEL_T","TIMETAG","INS_X_VEL","INS_Y_VEL","INS_Z_VEL",
% "INS_AZ","INS_ROLL","INS_PITCH","INS_TRUEHDG","INS_MAG_HDG","X_ACCEL","Y_ACCEL","Z_ACCEL","ROLL_RATE","PITCH_RATE","YAW_RATE",

% "MICRO-S","MICRO-S","FT/SEC","FT/SEC","FT/SEC","RAD","RAD","RAD","æSEC","FPS","FPS","FPS","DEG","ãRADS","ãRADS","ãRADS","ãRADS",
% "FPSý","FPSý","FPSý","ãRAD/S","ãRAD/S","ãRAD/S",

% "RAW 01","RAW 02","RAW 06","RAW 10","RAW 14","RAW 18","RAW 22","RAW 26","I01P 02","I01P 03","I01P 05","I01P 07","I01P 09","I01P 10",
% "I01P 11","I01P 12","I01P 13","I01P 14","I01P 15","I01P 16","I01P 30","I01P 31","I01P 32",

data_cell = textscan(h764_f, str, 'Delimiter',',','EmptyValue',0);
data_m = cell2mat (data_cell);

h764.ts     = data_m(:,1);                      % SYSTEM_TT, micro-sec
h764.t      = data_m(:,2);                      % GPS_TIME, micro-sec

dt   = median(diff(h764.t));
freq = round(1/dt);                             % Frequency

h764.fb         = data_m(:,3:5) * F2M * freq;   % X_DEL_V, Y_DEL_V, Z_DEL_V, ft/s
h764.wb         = data_m(:,6:8) * freq;         % X_DEL_T, Y_DEL_T, Z_DEL_T, rad
h764.timetag    = data_m(:,9);                  % TIMETAG
h764.vel        = data_m(:,10:12) * F2M;        % INS_X_VEL, INS_Y_VEL, INS_Z_VEL, ft/s
h764.az         = data_m(:,13) * D2R;           % INS_AZ, deg
h764.roll       = data_m(:,14);                 % INS_ROLL, rad
h764.pitch      = data_m(:,15);                 % INS_PITCH, rad
h764.yaw        = data_m(:,16);                 % INS_TRUEHDG, rad
h764.yawm       = data_m(:,17);                 % INS_MAG_HDG, rad
h764.acc        = data_m(:,18:20) * F2M;        % X_ACCEL, Y_ACCEL, Z_ACCEL, ft/s,
h764.gyro       = data_m(:,21:23);              % ROLL_RATE, PITCH_RATE, YAW_RATE, rad/s

h764.freq = freq;

% Header
h764.header = h764_h;

fclose(h764_f);

