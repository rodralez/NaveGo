function [distance, delta_pos] = gnss_distance_mercator (lat, lon, h)
% gnss_distance_mercator: provides the distance distance traveled by a 
% vehicle based on Mercator method and the WGS 84 Earth model.
%
% INPUT
%   lat, Mx1 latitudes (radians).
%   lon, Mx1 longitudes (radians).
%   h,   Mx1 altitude(meters).
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
%	Function provided by Github user RealMadridFK. 
% URL: https://github.com/rodralez/NaveGo/discussions/69
%
% Version: 001
% Date:    2021/10/08
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

R_equator = 6378137.000; % Earth's radius in meters

lat0 = lat(1);
lon0 = lon(1);
scale = cos(lat0);

R = R_equator * (0.99832407 + 0.00167644 * cos(2.0*lat0) - 0.00000352 * cos(4.0*lat0) );

x = scale * R * lon;
y = scale * R * log(tan(pi/4.0 + lat/2.0)) ;

x0 = scale * R * lon0;
y0 = scale * R * log(tan(pi/4.0 + lat0/2.0)) ;
x = (x - x0);
y = (y - y0);
z = (h - h(1));

% Cartesian coords
coord(:,1) = x(:,1);
coord(:,2) = y(:,1);
coord(:,3) = z(:,1);

% Сalculating distances between coordinate points
delta_x = [0; diff(coord(:,1))];
delta_y = [0; diff(coord(:,2))];
delta_z = [0; diff(coord(:,3))];

delta_pos = sqrt(delta_x.^2 + delta_y.^2 + delta_z.^2);

% Calculating full distance
distance = sum(delta_pos);
end
