function [vel_ned, acc_ned] = pllh2vned (ref)
% pllh2vned: generates NED accelerations and velocities from
% positions in the navigation frame.
%
% INPUT
%		ref: data structure with true trajectory.
%
% OUTPUT
%		vel_ned: Nx3 matrix with velocities in the NED frame [VN VE VD] (m/s, m/s, m/s).
%		acc_ned: Nx3 matrix with accelerations in the NED frame [AN AE AD] (m/s^2, m/s^2, m/s^2).
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
%
% Version: 002
% Date:    2022/05/15
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

% Method: LLH > ECEF > NED

ecef = llh2ecef([ref.lat ref.lon ref.h]);
eceforg = ecef(1,:);

[N,M] = size(ecef);
ned = zeros(N,M);

for i=1:N
    d_ecef  = ecef(i,:) - eceforg;
    ned(i,:)  = ecef2ned(d_ecef , [ref.lat(1) ref.lon(1) ref.h(1)]);
end

% Method: LLH > > NED

% [RM,RN] = radius(ref.lat);
% lat2m = RM+ref.h;
% lon2m = (RN+ref.h).*cos(ref.lat);
%
% latm = (ref.lat-ref.lat(1)) .* lat2m;
% lonm = (ref.lon-ref.lon(1)) .* lon2m;
%
% ned = [ latm lonm ref.h ];

% VEL
vel_raw =  diff(ned) ./ [diff(ref.t) diff(ref.t) diff(ref.t);];
vel_raw = [ 0 0 0; vel_raw; ];
% Smooth the noise from previous diff
vel_ned = my_sgolayfilt(vel_raw);
vel_ned = my_sgolayfilt(vel_ned);

% ACC
acc_raw = (diff(vel_ned)) ./ [diff(ref.t) diff(ref.t) diff(ref.t)];
acc_raw = [ 0 0 0; acc_raw; ];
% Smooth the noise from previous diff
acc_ned = my_sgolayfilt(acc_raw);
acc_ned = my_sgolayfilt(acc_ned);

end
