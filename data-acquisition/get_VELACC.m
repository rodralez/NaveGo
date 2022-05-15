function [week, tow, stat, vel_ned, acc_ned] = get_VELACC (line)
% get_VELACC: parses $VELACC field in .stat file from RTKPOST. 
% 
% INPUT
%   line, line from .stat file (string).
%
%   For example,
%       $VELACC,1845,383088.493,5,0.0156,-0.0143,0.0533,0.00000,0.00000,
%       0.00000,0.0000,0.0000,0.0000,0.00000,0.00000,0.00000
%
%   where,
%       $VELACC,week,tow,stat,vele,veln,velu,acce,accn,accu,velef,velnf,
%       veluf,accef,accnf,accuf
%
%       week/tow : gps week no/time of week (s)
%       stat : solution status (integer)
%       vele/veln/velu : velocity e/n/u (m/s) (float)
%       acce/accn/accu : acceleration e/n/u (m/s^2) (float)
%       velef/velnf/veluf : velocity e/n/u (m/s) (fixed)
%       accef/accnf/accuf : acceleration e/n/u (m/s^2) (fixed)
%
% OUTPUT
%   gnss_data, data structure with the following format:
%
%       week: GPS Week (integer)
%	    tow:  GPS Time Of Week (s)
%	    stat: Solution Status (integer)
%	    vel_ned: 1x3 NED velocity [n e d] (m/s, m/s, m/s)
%	    acc_ned: 1x3 NED accelerations [n e d] (m/s^2, m/s^2, m/s^2)
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
%   T. Takasu. RTKLIB ver. 2.4.2 Manual. April 29, 2013. 
%
% Version: 001
% Date:    2021/03/11
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

data = textscan(line,'%s %f%f%f %f%f%f %f%f%f %f%f%f %f%f%f',1,'delimiter',',');

week = data{2};
tow  = data{3};
stat = data{4};

vel_ned = [data{6} data{5} -data{7}];  % ENU to NED
acc_ned = [data{9} data{8} -data{10}]; % ENU to NED

end