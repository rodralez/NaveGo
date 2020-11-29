function ecef = llh2ecef(llh)
% llh2ecef: converts from navigation coordinates (latitude, longitude and 
% altitude) to ECEF coordinates.
%
% INPUTS
%   llh: Nx3 LLH coordinates [lat, lon, h] (rad, rad, m).
%
% OUTPUTS
%   ned: Nx3 NED coordinates [X Y Z] (m, m, m).
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
%   Guowei Cai et al. Unmanned Rotorcraft Systems. Springer. 2011. 
% Eq. 2.22, p. 31.
%
% Version: 001
% Date:    2019/01/16
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

% Preallocate

ecef = zeros(size(llh));

lat = llh(:,1);
lon = llh(:,2);
h   = llh(:,3);

[~,RN] = radius(lat);

e = 0.0818191908426;    % Eccentricity 

slat = sin(lat);
clat = cos(lat);
clon = cos(lon);
slon = sin(lon);

ecef(:,1) = (RN + h) .* clat .* clon;
ecef(:,2) = (RN + h) .* clat .* slon;
ecef(:,3) = (RN *(1-e^2) + h) .* slat;

end
