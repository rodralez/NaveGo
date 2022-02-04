function omega_ie_n = earth_rate(lat)
% earth_rate: turn rate of the Earth in the navigation frame.
%
% INPUT
%	lat, 1x1 latitude (rad).
%
% OUTPUT
%	omega_ie_n, 3x3 earth rate (rad/s).
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
%   Paul D. Groves. Principles of GNSS, Inertial, and
% Multisensor Integrated Navigation Systems. Second Edition.
% Eq. 5.41, page 177.
%
% Version: 002
% Date:    2022/01/25
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego 

omega_ie_n = (7.2921155e-5) .* [0 sin(lat) 0; ...
                                -sin(lat) 0  -cos(lat); ...
                                0 cos(lat) 0 ; ]; 
