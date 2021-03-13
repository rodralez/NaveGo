function [xsens , xsens_fields ] = xsens_read(file, row_lines, columns)
% xsens_read: reads .cvs files created by Xsens software.
%
% INPUT
%   fname: .cvs file name (string).
%   row_lines: number of rows in file.
%   columns: maximum column to be read.
%
% OUTPUT
%   xsens: data structure where each field is a column found in the file.
%   xsens_fields: cell vector with the names of each field found in the file.
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
% Version: 002
% Date:    2020/10/30
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

disp('xsens_read: processing data...')

%% OPEN FILE

xsens_f = fopen(file, 'r');
if xsens_f == -1
    error('xsens_read: ERROR: %s not found', file)
else
    fprintf('xsens_read: Loading %s...\n', file)
end

fprintf('xsens_read: %s has %d lines.\n', file, row_lines)

%% HEADER

header_rows = 13;

xsens_header = cell(header_rows,1);
for a = 1:header_rows
    xsens_header{a,1} = fgets(xsens_f);
end

%% FIELDS

xsens_line = fgets(xsens_f);
xsens_line = strrep(xsens_line,'[','_');    % Chance '[' for '_'
xsens_line = strrep(xsens_line,']','');     % Delete ']'

xsens_fields = textscan(xsens_line, '%s', 'Delimiter', ',' , 'EmptyValue', 0);

xsens_fields = xsens_fields{1};
xsens_fields(83) = {'GNSSLongitude'};
xsens_fields(84) = {'GNSSLatitude'};

[N,~] = size (xsens_fields);

fprintf('xsens_read: %s has %d columns.\n', file, N);

fprintf('xsens_read: only %d columns will be processed.\n', columns);

%% GET DATA

% Set pointer back to the beginning of file
% fseek(xsens_f,0,'bof');

% Pattern for textscan()
formatSpec = repmat('%f ', 1, N);

% Correction of numbers of rows
row_lines = row_lines - header_rows;

% Preallocation
xsens_imu = zeros(row_lines, columns);
xsens_gnss = zeros(floor(row_lines/2), columns);
xsens_ahrs = zeros(floor(row_lines/2), columns);
xsens_mag = zeros(floor(row_lines/2), columns);
xsens_att = zeros(floor(row_lines/2), columns);
xsens_baro = zeros(floor(row_lines/2), columns);

% Index for data allocation inside the while loop.
a = 1;  % index for reading IMU data
% b
c = 1;  % index for reading GNSS data
% d
e = 1;  % index for reading MAG data
% f
g = 1;  % index for reading AHRS data
% h
i = 1;  % index for reading ATTITUDE data
% j
k = 1;  % index for reading BARO data
% l

rows_block = 75000;		% Default vauel for rows_block

if rows_block > row_lines
    rows_block = floor(row_lines / 5);
end

fprintf('xsens_read: processing %s...\n', file)

loop_line = 1;
loop_ctr = 0;

while ~feof(xsens_f)
    
    fprintf('xsens_read: processing line %d...\n',  loop_line)
    loop_ctr = loop_ctr + 1;
    loop_line = loop_ctr * rows_block;
    
    data_c = textscan(xsens_f, formatSpec, rows_block, 'Delimiter',',');
    data = cell2mat(data_c(1:columns));
    
    % Delete rows where date is NaN for IMU data
    idl = ~isnan(data(: , 18));     % Column 18 is Acc_X
    data_imu = data(idl , :);
    
    % Delete rows where the difference between two adjacent times is negative
    idl = diff(data_imu(: , 2)) > 0;
    idl = [true; idl];
    data_imu = data_imu(idl , :);
    
    % Delete rows where date is NaN for GNSS data
    idl = ~isnan(data(: , 70));     % Column 70 is NUTimeOfWeek
    data_gnss = data(idl , :);
    
    % Delete rows where date is NaN for MAG data
    idl = ~isnan(data(: , 33));     % Column 33 is Mag_X
    data_mag = data(idl , :);
    
    % Delete rows where date is NaN for AHRS data
    idl = ~isnan(data(: , 24));     % Column 24 is AccHR_X
    data_ahrs = data(idl , :);
    
    % Delete rows where date is NaN for ATTITUDE data
    idl = ~isnan(data(: , 44));     % Column 44 is Quat_q0
    data_att = data(idl , :);
    
    % Delete rows where date is NaN for BARO data
    idl = ~isnan(data(: , 43));     % Column 43 is Pressure
    data_baro = data(idl , :);
    
    [M, ~] = size( data_imu );
    b = a + M - 1;
    xsens_imu(a:b , :) = data_imu;
    a = b + 1;
    
    [M, ~] = size( data_gnss );
    d = c + M - 1;
    xsens_gnss(c:d , :) = data_gnss;
    c = d + 1;
    
    [M, ~] = size( data_mag );
    f = e + M - 1;
    xsens_mag(e:f , :) = data_mag;
    e = f + 1;
    
    [M, ~] = size( data_ahrs );
    h = g + M - 1;
    xsens_ahrs(g:h , :) = data_ahrs;
    g = h + 1;
    
    [M, ~] = size( data_att);
    j = i + M - 1;
    xsens_att(i:j , :) = data_att;
    i = j + 1;
    
    [M, ~] = size( data_baro);
    l = k + M - 1;
    xsens_baro(k:l , :) = data_baro;
    k = l + 1;
    
    clear data_c data data_vld idl data_imu data_gnss data_mag data_ahrs
    
end

fclose(xsens_f);

% Delete epmty rows
idl = xsens_imu(: , 1) ~= 0.0 ;
xsens_imu = xsens_imu(idl , :);

idl = xsens_gnss(: , 1) ~= 0.0 ;
xsens_gnss = xsens_gnss(idl , :);

idl = xsens_mag(: , 1) ~= 0.0 ;
xsens_mag = xsens_mag(idl , :);

idl = xsens_ahrs(: , 1) ~= 0.0 ;
xsens_ahrs = xsens_ahrs(idl , :);

idl = xsens_att(: , 1) ~= 0.0 ;
xsens_att = xsens_att(idl , :);

idl = xsens_baro(: , 1) ~= 0.0 ;
xsens_baro = xsens_baro(idl , :);

%% MATRIX TO STRUCTURE

xsens = struct;

% IMU DATA
for a=1:23
    xsens.(xsens_fields{a}) = xsens_imu(:, a);
end

for a=27:29
    xsens.(xsens_fields{a}) = xsens_imu(:, a);
end

for a=36:42
    xsens.(xsens_fields{a}) = xsens_imu(:, a);
end

for a=60:66
    xsens.(xsens_fields{a}) = xsens_imu(:, a);
end

% AHRS DATA
for a=24:26
    xsens.(xsens_fields{a}) = xsens_ahrs(:, a);
end

for a=30:32
    xsens.(xsens_fields{a}) = xsens_ahrs(:, a);
end

% MAG DATA
for a=33:35
    xsens.(xsens_fields{a}) = xsens_mag(:, a);
end

% ATTITUDE DATA
for a=44:59
    xsens.(xsens_fields{a}) = xsens_att(:, a);
end

% BARO DATA
xsens.(xsens_fields{43}) = xsens_baro(:, 43);

% GNSS DATA
for a=70:columns
    xsens.(xsens_fields{a}) = xsens_gnss(:, a);
end

xsens.SampleTimeFine        = xsens_imu(:, 2);      
xsens.AHRSSampleTimeFine    = xsens_ahrs(:, 2);     
xsens.MagSampleTimeFine     = xsens_mag(:, 2);     
xsens.AttitudeSampleTimeFine = xsens_att(:, 2);    
xsens.BaroSampleTimeFine    = xsens_baro(:, 2);    
xsens.GNSSSampleTimeFine    = xsens_gnss(:, 2);

% FREQUENCIES
dti = median(diff(xsens.SampleTimeFine)) * 10e-5;
xsens.freq_imu = (1/dti);

dta = median(diff(xsens.AHRSSampleTimeFine)) * 10e-5;
xsens.freq_ahrs = (1/dta);

dtm = median(diff(xsens.MagSampleTimeFine)) * 10e-5;
xsens.freq_mag = (1/dtm);

dtt = median(diff(xsens.AttitudeSampleTimeFine)) * 10e-5;
xsens.freq_att = (1/dtt);

dtb = median(diff(xsens.BaroSampleTimeFine)) * 10e-5;
xsens.freq_baro = (1/dtb);

dtg = median(diff(xsens.NUTimeOfWeek)) * 10e-4;
xsens.freq_gnss = (1/dtg);

% Header
xsens.header = xsens_header;

disp('xsens_read: data is ready.')

end
