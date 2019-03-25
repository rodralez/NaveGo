function g_n = gravity(lat, h)
% gravity: calculates gravity vector in the navigation frame.
%
% INPUT:
%       lat: Mx1 latitude (radians).
%         h: Mx1 altitude (m).
%
% OUTPUT:
%		g_n: Mx1 gravity vector in the nav-frame (m/s^2).
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
%			Titterton, D.H. and Weston, J.L. (2004). Strapdown
% Inertial Navigation Technology (2nd Ed.). Institution
% of Engineering and Technology, USA. Eq. 3.89-3.91.
%
%  			R. Gonzalez, J. Giribet, and H. Patiño. An approach to
% benchmarking of loosely coupled low-cost navigation systems,
% Mathematical and Computer Modelling of Dynamical Systems, vol. 21,
% issue 3, pp. 272-287, 2015. Eq. 16.
%
% Version: 004
% Date:    2019/01/09
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

% Set gravity uncertainty
% a = 9.81 * 0.1;
% b = 9.81 * 0.1;
% g_noise = (b-a).*rand(1) + a;

h = abs(h);
sin1 = sin(lat);
sin2 = sin(2.*lat);

g0 = 9.780318 * ( 1 + 5.3024e-03.*(sin1).^2 - 5.9e-06.*(sin2).^2 );

[RM,RN] = radius(lat);

Ro = sqrt(RN .* RM);

g = (g0 ./ (1 + (h ./ Ro)).^2);

Z = zeros(size(lat));

g_n = [Z Z g];

end

