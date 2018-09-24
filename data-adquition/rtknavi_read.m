function gnss = rtknavi_read (fname)
% rtknavi_read: read RTKAVI output file and transforms it to NaveGo format.
%
% INPUT:
%   fname, file name (string).
%
% OUTPUT
%   gnss_data, data structure withe following format:
%
%	week, GPS Week
%	t, GPS Time Of Week (s)
%	stat, Solution Status
%	ecef, ECEF position (m)
%	vel, NED velocities (m/s)
%	acc, NED accerelations (m/s^2) 
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
%   
%       T. Takasu. RTKLIB ver. 2.4.2 Manual. April 29, 2013. 
%
% Version: 001
% Date:    2018/09/11
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

D2R = pi/180;

%% OPEN FILE

fid = fopen(fname, 'r');
if fid == -1
    error('rtknavi_read: %s file not found', fid)
end

%% TOTAL NUMBER OF LINES

lines = nnz(fread(fid) == 10);
fprintf('rtknavi_read: %s file has %d lines. \n', fname, lines);

% Set pointer back to the beginning
fseek(fid,0,'bof');

%% DATA PREALLOCATION

% Total number of columns
COL = 28;

gnss_data = zeros(lines, COL);

%% INITILIZE DATA

vld_flag = 0;
row = 2;

% POS
week = 0;
tow = 0;
stat = 0;
ecef = zeros(1,3);

% VELACC
vel_ned = zeros(1,3);
acc_ned = zeros(1,3);

% CLK
rcv = 0;
clkbias_gps = 0;
clkbias_glns  = 0;

%SAT
satid = 0;
satfreq = 0;
azm = 0;
psrange = 0;
cphase = 0;
vsat = 0;
snr = 0;
amb_flag = 0;
slip_flat = 0;
lock_cnt = 0;
out_cnt = 0;
slip_cnt = 0;
rejc_cnt = 0;

%% PROCESS FILE

TOW = 2;

while ~feof(fid)
    
    line = fgetl(fid); 
    nmea = sscanf (line, '%c', 4);
    
    switch nmea
        
        case '$POS'
            
            [week, tow, stat, ecef] = get_POS (line);
            vld_flag = 1;
            
        case '$VEL'
            
            [week, tow, stat, vel_ned, acc_ned] = get_VELACC (line);
            vld_flag = 1;
            
        case '$CLK'
            
            [week, tow, stat, rcv, clkbias_gps, clkbias_glns] = get_CLK (line);
            vld_flag = 1;
            
        case '$SAT'
            
            continue
%             [week, tow, satid, satfreq, azm, psrange, cphase, vsat, snr, ...
%             amb_flag, slip_flat, lock_cnt, out_cnt, slip_cnt, rejc_cnt] = get_SAT (line);
%             vld_flag = 1;
            
        otherwise
            
            fprintf('rtknavi_read: unknown type of line. \n');
            vld_flag = 0;            
    end

    if (vld_flag == 1)
        
        % Check if two is the same
        if (gnss_data(row-1, TOW) ==  tow)
            row = row - 1;
        end
        
        gnss_data(row,:) = [week, tow, stat, ecef, vel_ned, acc_ned, rcv, clkbias_gps, ...
            clkbias_glns, satid, satfreq, azm, psrange, cphase, vsat, snr, ...
            amb_flag, slip_flat, lock_cnt, out_cnt, slip_cnt, rejc_cnt];
        
        row = row + 1;
        vld_flag = 0;
    end
end

fclose (fid);

fprintf('rtknavi_read: end of file. \n');
    
%% TOW DATA
% GPS time of week
tow = gnss_data(:, TOW);                                 

% Index with valid tow data
iix = find (tow > 500);

% Max tow value within valid tow
M = max(tow(iix));
% Index of Max tow

ffx = find (tow(iix) == M, 1, 'first');

% Span valid index from element 1 to index of Max tow
idx = iix (1:ffx);

gnss.week  = gnss_data(idx,1);      % GPS Week
gnss.t     = gnss_data(idx,2);   	% GPS Time Of Week (s)
gnss.stat  = gnss_data(idx,3);      % Solution Status
gnss.ecef  = gnss_data(idx,4:6);    % ECEF position (m)
gnss.vel   = gnss_data(idx,7:9);    % NED velocities (m/s)
gnss.acc   = gnss_data(idx,10:12);  % NED accerelations (m/s)

dtg = median(diff(gnss.t));
gnss.freq = 1/dtg;

% gnss.sfreq   = gnss_data(idx,13);
% gnss.azm     = gnss_data(idx,14);
% gnss.psrange = gnss_data(idx,15);
% gnss.cphase  = gnss_data(idx,16);
% gnss.vsat    = gnss_data(idx,17);
% gnss.snr     = gnss_data(idx,18);
% gnss.amb_flag = gnss_data(idx,19);
% gnss.slip_flat= gnss_data(idx,20);
% gnss.lock_cnt = gnss_data(idx,21);
% gnss.out_cnt  = gnss_data(idx,22);
% gnss.slip_cnt = gnss_data(idx,23);
% gnss.rejc_cn  = gnss_data(idx,24);

end
