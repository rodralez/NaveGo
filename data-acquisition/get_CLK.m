function [week, tow, stat, rcv, clk_gps, clk_glns] = get_CLK (line)
% get_CLK: parses $CLK field in .stat file from RTKPOST. 
% 
% INPUT
%   line, line from .stat file (string).
%
%   For example,
%       $CLK,1845,383088.493,5,1,7139202.842,7139202.842,0.000,0.000
%
%   where,
%       $CLK,week,tow,stat,rcv,clk1,clk2,clk3,clk4
%
%       week/tow : gps week no/time of week (s)
%       stat : solution status
%       rcv : receiver (1:rover,2:base station)
%       clk1 : receiver clock bias GPS (ns)
%       clk2 : receiver clock bias GLONASS (ns)
%       clk3 : reserved
%       clk4 : reserved
%
% OUTPUT
%   gnss_data, data structure with the following format:
%       week: GPS Week (integer)
%	    tow:  GPS Time Of Week (s)
%	    stat: Solution Status (integer)
%	    rcv: receiver (1:rover,2:base station)
%	    clk_gps: receiver clock bias GPS (ns)
%       clk_glns: receiver clock bias GLONASS (ns)
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

data = textscan(line,'%s%f%f%f %f %f %f ',1,'delimiter',',');

week = data{2};
tow  = data{3};
stat = data{4};

rcv = data{5};

clk_gps  = data{6};
clk_glns = data{7};

end