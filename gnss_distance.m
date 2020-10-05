function [d, dp] = gnss_distance (lat, lon)
% gnss_distance: provides distance between geographic coordinates based on 
% haversine function.
%
% INPUT:
%       lat: Mx1 latitudes (radians).
%       lon: Mx1 longitudes (radians).
%
% OUTPUT:
%		d: total distance in meters.
%       dp: Mx1 incremental distances between near points in meters.
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
% Version: 002
% Date:    2020/09/03
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

R = 6.3781 * 10^6;              % Earth's radius in m

delta_lat = diff ( lat );   
delta_lat = [0; delta_lat];

delta_lon = diff ( lon );
delta_lon = [0; delta_lon];

a = sin( delta_lat ./ 2 ).^2 + cos( lat ).* cos( lon ) .* ...
        sin( delta_lon ./ 2 ).^2;

c = 2 .* atan2 ( sqrt(a), sqrt (1-a) );

dp = R .* c;

d = sum(dp);

end

