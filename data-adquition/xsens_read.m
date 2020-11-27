function [xsens , xsens_fields ] = xsens_read(file, row_lines, columns)
% xsens_read: reads .cvs files created by Xsens software.
%
% INPUT
%   fname: .cvs file name (string).
%   blockMatrix:
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
% Version: 003
% Date:    2020/11/27
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

fprintf('xsens_read: %s has %d lines.\n', row_lines)

%% HEADER

header_rows = 13;

xsens_header = cell(header_rows,1);
for i = 1:header_rows
    xsens_header{i,1} = fgets(xsens_f);
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

% Init data matrix
row_lines = row_lines - header_rows;
xsens_imu = zeros(row_lines, columns);
xsens_gnss = zeros(floor(row_lines/2), columns);

i = 1;  % index for reading IMU data
h = 1;  % index for reading GNSS data

rows_block = 75000;

if rows_block > row_lines
    rows_block = floor(row_lines / 5);
end

fprintf('xsens_read: processing %s...\n', file)

while ~feof(xsens_f)

    fprintf('xsens_read: processing line %d...\n', i)
        
    data_c = textscan(xsens_f, formatSpec, rows_block, 'Delimiter',','); % ,'EmptyValue',0
    data = cell2mat(data_c(1:columns));
    
    % Delete rows where date is NaN for IMU data
    idl = ~isnan(data(: , 3));
    data_imu = data(idl , :);
    
    % Delete rows where date is NaN for GNSS data
    idl = ~isnan(data(: , 70));
    data_gnss = data(idl , :);
        
    % Delete rows where the difference between two adjacent times is negative
    idl = diff(data_imu(: , 2)) > 0;
    idl = [true; idl];
    data_imu = data_imu(idl , :);
   
    [M, ~] = size( data_imu );    
    j = i + M - 1;
    xsens_imu(i:j , :) = data_imu;
    i = j + 1;

    [M, ~] = size( data_gnss );    
    k = h + M - 1;
    xsens_gnss(h:k , :) = data_gnss;
    h = k + 1;
    
    clear data_c data data_vld idl

end

% Delete epmty rows
idl = xsens_imu(: , 1) ~= 0.0 ;
xsens_imu = xsens_imu(idl , :);

idl = xsens_gnss(: , 1) ~= 0.0 ;
xsens_gnss = xsens_gnss(idl , :);

%% DATA IN STRUCTURE

xsens = struct;

for i=1:69
    xsens.(xsens_fields{i}) = xsens_imu(:, i);
end

for i=70:columns
    xsens.(xsens_fields{i}) = xsens_gnss(:, i);
end

xsens.GNSSSampleTimeFine = xsens_gnss(:, 2);
 
% Frequencies
dti = median(diff(xsens.UTC_Nano)) * 10e-10;
xsens.freq_imu = (1/dti);

dtg = median(diff(xsens.NUTimeOfWeek)) * 10e-4;
xsens.freq_gnss = (1/dtg);

% Header
xsens.header = xsens_header;

fclose(xsens_f);

disp('xsens_read: data is ready.')

end
