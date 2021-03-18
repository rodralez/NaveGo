function gn = gravity(lat, h)
% gravity: calculates gravity vector in the navigation frame.
%
% INPUT
%       lat, Mx1 latitude (radians).
%         h, Mx1 altitude (m).
%
% OUTPUT
%		g_n, Mx3 gravity vector in the nav-frame (m/s^2).
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
%   Paul D. Groves. Principles of GNSS, Inertial, and
% Multisensor Integrated Navigation Systems. Second Edition.
% Eq. 2.134 to 2.140, page 70.
%
% Version: 004
% Date:    2021/03/17
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

M = max(size(lat));

gn = zeros(M,3);

% Parameters
R_0 = 6378137;              % WGS84 Equatorial radius in meters
R_P = 6356752.31425;        % WGS84 Polar radius in meters
e = 0.0818191908425;        % WGS84 eccentricity
f = 1 / 298.257223563;      % WGS84 flattening
mu = 3.986004418E14;        % WGS84 Earth gravitational constant (m^3 s^-2)
omega_ie_n = 7.292115E-5;   % Earth rotation rate (rad/s)

% Calculate surface gravity using the Somigliana model, (2.134)
sinsqL = sin(lat).^2;
g_0 = 9.7803253359 * (1 + 0.001931853 .* sinsqL) ./ sqrt(1 - e^2 .* sinsqL);

% Calculate north gravity using (2.140)
gn(:,1) = -8.08E-9 .* h .* sin(2 .* lat);

% East gravity is zero
gn(:,2) = 0;

% Calculate down gravity using (2.139)
gn(:,3) = g_0 .* (1 - (2 ./ R_0) .* (1 + f .* (1 - 2 .* sinsqL) +...
    (omega_ie_n^2 .* R_0^2 .* R_P ./ mu)) .* h + (3 .* h.^2 ./ R_0^2));

end

