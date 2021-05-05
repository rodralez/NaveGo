function sensor_data = sensor_read(file_name, head_lines, fields_line, specific_delimiter)
% sensor_read: tries to read .cvs files from any type of sensor.
%
% INPUT
%   file_name, name of the .cvs file (string).
%   head_lines, number of lines for the file header.
%   fields_line, number of the particular line in the file where the label 
%       of each column data is provided. field_line has to be inside the file 
%       header.
%   specific_delimiter, character delimiter used in the .csv file. Default 
%       value is a comma (char).
%
% OUTPUT
%   sensor_data, data structure where each field is a column found in the 
%       provide .csv file.
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
% Date:    2021/05/05
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego 

%% INPUT CHECKING

if nargin < 4
    specific_delimiter = ',';
end

% If field_line is outside of the header...
if fields_line > head_lines
    
    error('sensor_read: field_line is outside of the file header.')
end

%% HEADER

sensor_f = fopen(file_name, 'r');

if sensor_f == -1
    error('sensor_read: file %s not found', file_name)
else
    fprintf('sensor_read: reading data from file %s. \n', file_name)
end

sensor_header = cell(head_lines, 1);

for j=1:head_lines
    
    sensor_header{j} = fgets(sensor_f);
end

%% FIELDS

sensor_fields_raw = textscan(sensor_header{fields_line, :}, '%s', ... 
    'EmptyValue', 0, 'Delimiter', specific_delimiter);

sensors_fields = sensor_fields_raw{1,:};

% The names of the fields has to be changed according to MATLAB guidelines 
% for variable names

% Any illegal starting character in fields is changed by 'NC_'
legal_init_char = 'a-zA-Z';
reg_pattern = sprintf( '^[^%s]?', legal_init_char);
sensors_fields = regexprep(sensors_fields, reg_pattern, 'NC_');

% Any illegal character in fields is changed by an underscore
legal_char = '\W';              % \W equals to [^a-zA-Z0-9_]
reg_pattern = sprintf( '[%s]', legal_char);
sensors_fields = regexprep(sensors_fields, reg_pattern, '_');

columns = max (size ( sensors_fields ) );

fprintf('sensor_read: %d columns are detected.\n', columns)

%% DATA READING

data_cell = textscan( sensor_f, '%f' , 'Headerlines' , head_lines, 'Delimiter', specific_delimiter );

data_raw = data_cell{1,1};

M = max(size(data_raw));

% NaN values are forced to be zero
inx = isnan(data_raw);
data_raw(inx) = 0.0;

rows = M/columns;

data_raw = reshape (data_raw, columns, rows);

data_raw = data_raw';

%% DATA TO STRUCTURE

sensor_data = struct;

% Data
for i=1:columns
    
    sensor_data.(sensors_fields{i}) = data_raw(:, i);
end

% Header
sensor_data.header = sensor_header;

% Raw names of fields
sensor_data.raw_fields = sensor_fields_raw{1,:};

%%

fclose(sensor_f);

disp('sensor_read: processing is finished.')

end
