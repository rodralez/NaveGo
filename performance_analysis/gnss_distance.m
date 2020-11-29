function [distance, delta_pos] = gnss_distance (lat, lon)
% gnss_distance: provides the distance distance traveled by a vehicle based 
% on haversine function.
%
% INPUT:
%   lat: Mx1 latitudes (radians).
%   lon: Mx1 longitudes (radians).
%
% OUTPUT:
%	distance: total distance (meters).
%   delta_pos: Mx1 incremental distances between near points (meters).
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
%
%	Ritchie Smith. Harversine function in Mathworks File Exchange. 
% URL: https://www.mathworks.com/matlabcentral/fileexchange/32883-haversine
%
% Version: 003
% Date:    2020/11/19
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

R = 6.3781 * 10^6;              % Earth's radius in m

delta_lat = [ 0; diff(lat) ];

delta_lon = [ 0; diff(lon) ];

a = sin( delta_lat ./ 2 ).^2 + cos( lat ).* cos( lon ) .* ...
        sin( delta_lon ./ 2 ).^2;

c = 2 .* atan2 ( sqrt(a), sqrt (1-a) );

delta_pos = R .* c;

distance = sum(delta_pos);

end

