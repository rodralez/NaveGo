function [d, d_v] = gnss_distance (lat, lon)
% gnss_distance: provides distance between geographic coordinates based on 
% haversine function.
%
% INPUT:
%       lat: Mx1 latitudes (radians).
%       lon: Mx1 longitudes (radians).
%
% OUTPUT:
%		d:  total distance in kilometeres.
%       d_v: M-1x1 incremental distances between near points in kilometeres.
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
% References:
%			Ritchie Smith. Harversine function in Mathworks File Exchange. 
% URL: https://www.mathworks.com/matlabcentral/fileexchange/32883-haversine
%
% Version: 001
% Date:    2020/03/04
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

M = max ( size ( lat) );

d_v = zeros(M-1, 1);            % Incremental distances between near points

R = 6371;                       % Earth's radius in km

delta_lat = diff ( lat );   
    
delta_lon = diff ( lon );
    
for i = 1:M-1
   
    a = sin( delta_lat(i) / 2)^2 + cos( lat(i) )  * cos( lon(i) ) * ...
    sin( delta_lon(i) / 2)^2;

    c = 2 * atan2 ( sqrt(a), sqrt (1-a) );

    d_v(i) = R * c;                          % distance in km
end

d = sum(d_v);

