function xsens = xsens_read(file)
% xsens_read: reads .cvs files created by Xsens software. 
%
% INPUT
%   fname: .cvs file name (string).
%
% OUTPUT
%   xsens: data structure which fields depends on user selection. 
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
%
% Version: 001
% Date:    2020/10/22
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

%% CONSTANTS

G =   9.80665;
D2R = pi/180;

disp('xsens_read: processing data...')

%% OPEN FILE

xsens_f = fopen(file, 'r');
if xsens_f == -1
    error('xsens_read: ERROR: %s not found', file)
end

%% TOTAL NUMBER OF LINES

lines = nnz(fread(xsens_f) == 10);
fprintf('xsens_read: %s file has %d lines. \n', file, lines);

% Set pointer back to the beginning of file
fseek(xsens_f,0,'bof');

%% DATA STRUCTURE

xsens = struct;
xsens_fields = {'packet_counter','year','month','day','second','UTC_nano', ...
                'UTC_year','UTC_month','UTC_day', 'UTC_hour','UTC_minute', ...
                'UTC_second','UTC_valid','vel_inc_X','vel_inc_Y','vel_inc_Z', ...
                'ori_inc_q0','ori_inc_q1','ori_inc_q2','ori_inc_q3'};

N = max(size(xsens_fields));

for i=1:N
    xsens.(xsens_fields{i}) = 0;
end

%% HEADER

header_rows = 14;

xsens_h = cell(header_rows,1);
for i = 1:header_rows
    xsens_h{i,1} = fgets(xsens_f);
end

% Set pointer back to the beginning of file
fseek(xsens_f,0,'bof');

%% DATA

% Pattern for textscan()
str = repmat('%f ', 1, N);

data_cell = textscan(xsens_f, str, 'HeaderLines', 14, 'Delimiter',',','EmptyValue',0);
data_m = cell2mat (data_cell);

%% IMU data

% Delete repeated timestamps
dd = diff(data_m(:, 6));
dd = [1; dd];
idl = dd ~= 0.0;
data_vld = data_m(idl , :);

if (~ all (idl))
    warning ('Repeated IMU timestamp will be deleted')
end

xsens.packet_counter    = data_vld(:, 1);

xsens.year              = data_vld(:, 2);
xsens.month             = data_vld(:, 3);
xsens.day               = data_vld(:, 4);
xsens.second            = data_vld(:, 5);

xsens.UTC_nano          = data_vld(:, 6);
xsens.UTC_year          = data_vld(:, 7);
xsens.UTC_month         = data_vld(:, 8);
xsens.UTC_day           = data_vld(:, 9);
xsens.UTC_hour          = data_vld(:, 10);
xsens.UTC_minute        = data_vld(:, 11);
xsens.UTC_second        = data_vld(:, 12);

xsens.UTC_valid         = data_vld(:, 13);

xsens.vel_inc_X         = data_vld(:, 14);
xsens.vel_inc_Y         = data_vld(:, 15);
xsens.vel_inc_Z         = data_vld(:, 16);

xsens.ori_inc_q0        = data_vld(:, 17);
xsens.ori_inc_q1        = data_vld(:, 18);
xsens.ori_inc_q2        = data_vld(:, 19);
xsens.ori_inc_q3        = data_vld(:, 20);

% Frequency
dt = median(diff(xsens.UTC_nano)) * 10e-9;
xsens.freq = round(1/dt);

% Header
xsens.header = xsens_h;

% Check and correct if gravity is negative in NED coordinates.
% xsens = correct_gravity(xsens);

fclose(xsens_f);

disp('xsens_read: data is ready.')

end
