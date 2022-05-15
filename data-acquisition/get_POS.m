function [week, tow, stat, ecef] = get_POS (line)
% get_POS: parses $POS field in .stat file from RTKPOST. 
% 
% INPUT
%   line, line from .stat file (string).
%
%   For example,
%       $POS,1845,383088.493,5,4472642.4224,601346.1910,4492468.6195,0.0000,0.0000,0.0000
%
%   where,
%       $POS,week,tow,stat,posx,posy,posz,posxf,posyf,poszf 
%
%       week/tow : gps week no/time of week (s) 
%       stat : solution status 
%       posx/posy/posz : position x/y/z ecef (m) float 
%       posxf/posyf/poszf : position x/y/z ecef (m) fixed 
%
% OUTPUT
%   gnss_data, data structure with the following format:
%
%	    week: GPS Week (integer).
%	    tow:  GPS Time Of Week (s).
%	    stat: Solution Status (integer).
%	    ecef: ECEF position (m).
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

data = textscan(line,'%s%f%f%f%f%f%f%f%f%f',1,'delimiter',',');

week = data{2};
tow  = data{3};
stat = data{4};
ecef = cell2mat(data(5:7));



