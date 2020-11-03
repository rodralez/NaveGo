function [xsens_data, xsens ] = xsens_read(file, row_lines, columns_max)
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
    fprintf('xsens_read: loading %s...\n', file)
end

%% HEADER

header_rows = 13;

xsens_h = cell(header_rows,1);
for i = 1:header_rows
    xsens_h{i,1} = fgets(xsens_f);
end

%% FIELDS

xsens_line = fgets(xsens_f);
xsens_line = strrep(xsens_line,'[','_');    % Chance '[' for '_'
xsens_line = strrep(xsens_line,']','');     % Delete ']'

xsens_fields = textscan(xsens_line, '%s', 'Delimiter', ',' , 'EmptyValue', 0);

xsens_fields = xsens_fields{1};

[columns,~] = size (xsens_fields);

xsens.fields = xsens_fields;

fprintf('xsens_read: %s has %d columns. \n', file, columns);

%% GET DATA

% Set pointer back to the beginning of file
% fseek(xsens_f,0,'bof');

% Pattern for textscan()
formatSpec = repmat('%f ', 1, columns);

% Init data matrix
xsens_data = zeros(row_lines, columns_max);

j = 1;  % index for reading data

m = 0;  % index for dividing data in several matrices

blockRows = 10000;

while ~feof(xsens_f)
    
    fprintf('xsens_read: processing row %d...\n', j)
    
    data_c = textscan(xsens_f, formatSpec, blockRows, 'Delimiter',',','EmptyValue',0);
    data = cell2mat(data_c);
    
    % Avoid rows where time is NaN
    idl = isnan(data(:, 7));
    data_vld = data( ~idl , :);
    
    % Avoid rows where acc is NaN
    idl = ~isnan(data_vld(:, 18));
    data_vld = data_vld(idl , :);    
    
    [M, ~]  = size( data_vld );
    
    k = j + M - 1;
    xsens_data(j:k , :) = data_vld(: , 1:columns_max);
    j = k + 1;
    
    clear data_c data data_vld idl    
end

%% DATA TO STRUCTURE

% for i=1:columns_max
%     xsens.(xsens_fields{i}) = xsens_data(:, i);
% end

%% DATA TO TABLE

% xsens_table = array2table(xsens_data, 'VariableNames', xsens_fields(1:66)) ;

%%

% Delete extra rows where time is zero
idl = (xsens_data(:, 7)) == 0.0;
xsens_data = xsens_data( ~idl , :);

% Frequency
time = xsens_data( ~isnan(xsens_data(:, 7)) , 7);
dt = median( diff(time) ) * 10e-10;
xsens.freq = round(1/dt);

% Header
xsens.header = xsens_h;

% Check and correct if gravity is negative in NED coordinates.
% xsens = correct_gravity(xsens);

fclose(xsens_f);

disp('xsens_read: finishing processing.')

end
