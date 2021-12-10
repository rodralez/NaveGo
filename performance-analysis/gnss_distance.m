function [distance, delta_pos] = gnss_distance (lat, lon)
% gnss_distance: provides the distance distance traveled by a vehicle based 
% on haversine function and the WGS 84 Earth model.
%
% INPUT
%   lat, Mx1 latitudes (radians).
%   lon, Mx1 longitudes (radians).
%
% OUTPUT
%	distance, total distance (meters).
%   delta_pos, Mx1 incremental distances between near points (meters).
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
% Version: 005
% Date:    2021/09/17
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

% referenceEllipsoid('WGS 84')
%                 Code: 7030
%                  Name: 'WGS 84'
%            LengthUnit: 'meter'
%         SemimajorAxis: 6378137
%         SemiminorAxis: 6356752.31424518
%     InverseFlattening: 298.257223563
%          Eccentricity: 0.0818191908426215        

R = sqrt(6378137 * 6356752.31424518); 

delta_lat = [ 0; diff(lat) ];

delta_lon = [ 0; diff(lon) ];

a = sin( delta_lat ./ 2 ).^2 + cos( lat ).* cos( lon ) .* ...
        sin( delta_lon ./ 2 ).^2;

% Ensure that a falls in the closed interval [0 1].
a(a < 0) = 0;
a(a > 1) = 1;

c = 2 .* atan2 ( real(sqrt(a)), real(sqrt(1-a)) );

delta_pos = R .* c;

distance = sum(delta_pos);

end

