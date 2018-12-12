function gnss = rinex_read(fname)
% rinex_read: reads data from RINEX observation file (v2.11) and organize
% them into a particular data structure with raw GPS data, i.e., pseudo 
% ranges, carrier phase and pseudo-range rates.
%
% This function assumes that the Teqc program is installed on the 
% current file system and its instalation directory is available on the
% system path.
%
% L1 and C1 data are taken from GPS system only.
%
% INPUT:
%   fname, RINEX file name (string).
%
% OUTPUT
%   gnss, data structure with the following format. 
%
%     - raw, Nx15 matrix with rinex data ordered into columns as:
%           1: year.
%           2: month.
%           3: day.
%           4: hour.
%           5: minutes.
%           6: seconds.
%           7: number of satellite in view.
%           8: C1 pseudo-range using C/A Code on L1 (meters).
%           9: C1 pseudo-range signal strength (0-9) where, 
%               0 or blank: not known, don't care. 
%               1: minimum possible signal strength.          
%               5: threshold for good S/N ratio.              
%               9: maximum possible signal strength.          
%           10: L1 phase measurements on L1 (full cycles).
%           11: L1 phase signal strength (0-9).
%           12: pseudo-range rate on L1 (meters/s).
%       
%     - header, RINEX file header.   
%
% Each epoch in the raw matrix should have information for several satellites.
%
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
%   References:
%  
%       UNACOV. Teqc Tutorial, Basics of Teqc Use and Teqc Products. June 6, 
%       2014. 
%
%       Mark Petovello. How does a GNSS receiver estimate velocity?
%       InsideGNSS Magazine. March/April 2015.
%
%       Werner Gurtner. RINEX: The Receiver Independent Exchange Format
%       Version 2.11 (v2). June 26, 2012.
%   
%       Paul Groves, Principles of GNSS, Inertial, and
%       Multisensor Integrated Navigation Systems (2008). Chapter 7.
%
% Version: 001
% Date:    2018/12/07
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

%% CONSTANTS

L1 = 1575.42;       % L1 carrier frequency (Hz)
L2 = 1227.60;       % L2 carrier frequency (Hz)
C  = 299792458;     % Speed of light (m/s)

%% RINEX FILE

fid = fopen(fname, 'r');
if fid == -1
    error('rinex_read: %s file not found', fname)
end

% TOTAL NUMBER OF LINES

lines = nnz(fread(fid) == 10);
fprintf('rinex_read: %s file has %d lines. \n', fname, lines);

% Set pointer back to the beginning
fseek(fid,0,'bof');

% HEADER

hdlines = 0;
idx = [];
header = {};

while ( isempty(idx) )
    
    line = fgetl(fid);
    idx = find(contains(line,'END OF HEADER'));
    hdlines = hdlines + 1;
    header(hdlines, :) = cellstr (line);
end

fprintf('rinex_read: %s has %d header lines. \n', fname, hdlines);

% Set pointer back to the beginning
fseek(fid,0,'bof');

% SEARCH FOR INTERVAL TIME IN HEADER

flag = true;

while (flag)
    
    line = fgetl(fid);
    flag = ~ (contains(line,'INTERVAL'));
end

line_h_time = strsplit(line);
dt = str2double (line_h_time(2));   % INTERVAL

fprintf('rinex_read: interval time is %f. \n', dt);

% Set pointer back to the beginning
fseek(fid,0,'bof');

fclose(fid);

%% TEQC COMMAND

% -G elimate all GPS SVs and data *
% -R elimate all GLONASS SVs and data *
% -E elimate all Galileo/GPS SVs and data *
% -S elimate all SBAS SVs and data *
% -C elimate all Beidou­2/Compass SVs and data *
% -J elimate all QZSS SVs and data *

% L1, L2: Phase measurements on L1 and L2
% C1, C2: Pseudorange using C/A Code on L1,L2
% P1, P2: Pseudorange using P-Code on L1,L2
% D1, D2: Doppler frequency on L1 and L2
% T1, T2: Transit Integrated Doppler on 150 (T1) and 400 MHz (T2)
% S1, S2: Raw signal strengths or SNR values as given by the receiver for the L1,L2 phase observations

file_teqc = sprintf('temp_%s', fname);

fprintf('rinex_read: creating %s file with C1 and L1 data. \n', file_teqc);

command = sprintf('teqc -O.obs C1L1 -R -E -S ./%s > %s', fname, file_teqc);

[status,~] = system(command);
if status ~= 0
    error('rinex_read: teqc command fails')
end

%% TEQC FILE HEADER

fid = fopen(file_teqc, 'r');
if fid == -1
    error('rinex_read: %s file not found', file_teqc)
end

% SEARCH FOR TIME OF FIRST OBS
flag = true;

while (flag)
    
    line = fgetl(fid);
    flag = ~ (contains(line,'TIME OF FIRST OBS'));
end

line_h_time = strsplit(line);
year = cell2mat (line_h_time(2));

% SEARCH FOR FIRST ROW OF DATA

flag = true;

while (flag)
    
    line = fgetl(fid);
    flag = ~ (contains(line, year(3:4)));
end

% Set pointer back one row
fseek(fid,-1,'cof');

%% TEQC FILE BODY

data = zeros (lines, 12);
row = 1;

fprintf('rinex_read: getting RINEX data from body file...\n');

while ~feof(fid)
    
    line = fgetl(fid);
    
    % Take epoch time
    line_p_time = strsplit(line);
    line_size = max(size(line_p_time));
    
    % Check time line integrity
    if line_size > 8
        
        % Fill data with epoch time
        for idx = 2:7
            data (row, idx-1) = str2double ( cell2mat (line_p_time(idx)) );
        end
        
        % Number of satellites in epoch
        sat_num = strsplit(line_p_time{9},'G');
        sat_total = str2double(sat_num{1});
        
        % For each satellite in view, fill one row with satellite data
        for idx = 1:sat_total
            
            data (row, 7) = str2double(sat_num{idx+1});     % Satellite number
            line = fgetl(fid);
            
            if ~isempty(line)
                
                line_p_data = textscan(line,'%f %f %f %f');  % C1 L1
                
                for jdx = 1:4
                    n = line_p_data{jdx};
                    if ~isempty(n)
                        data (row, jdx+7) = n;
                    end
                end
            end
            
            row = row + 1;
        end
    end    
end

fclose(fid);

%% PSEUDO-RANGE RATES

fprintf('rinex_read: calculating pseudo-range rates... \n');

sat_v = nonzeros(unique(data(:,7)));

M = length(sat_v);

for k = 1:M
   
    sat_num = sat_v(k);
    
    idx = (data(:,7) == sat_num);
    
    phase_l1 = data(idx,10);
    
    % [Petovello, Eq. 8]
    doppler_l1 = diff (phase_l1) ./ dt;
    
    % [Groves, Eq. 7.129]
    prr_l1 = - doppler_l1 .* C ./ L1 ; 
    
    % Since prr_l1 has one less element than phase_l1, first row with sat
    % data should not be addressed
    
    % Find the first row with sat data
    jdx = find (idx == true, 1, 'first');
    
    idx(jdx) = false;
    
    data(idx,12) = prr_l1;
    
end

%% 

% Detect non-empty rows
idx = data(:, 7) ~= 0;

% Delete empty rows
data = data(idx, :);

gnss.raw = data;

gnss.header = header;

%% QUALITY

c1_ss = data(:,9);
c1_ss_median = median(c1_ss);
c1_ss_mean = mean(c1_ss);

fprintf('rinex_read: median of C1 pseudo-ranges signal strength is %f...\n', c1_ss_median);
fprintf('rinex_read: median of C1 pseudo-ranges signal strength is %f...\n', c1_ss_mean);

l1_ss = data(:,11);
l1_ss_median = median(l1_ss);
l1_ss_mean = mean(l1_ss);

fprintf('rinex_read: median of L1 phase measurements signal strength is %f...\n', l1_ss_median);
fprintf('rinex_read: median of L1 phase measurements signal strength is %f...\n', l1_ss_mean);


