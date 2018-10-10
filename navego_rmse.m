function rmse_v = navego_rmse (nav_e, gnss, ref_n, ref_g)
% print_rmse: print on console Root Mean Squared Errors (RMSE) between INS/GNSS
% and a reference, and between GNSS-only and a reference as well.
%
% INPUT:
%   nav_e, INS/GNSS integration data structure.
%   gnss, GNSS data structure.
%   ref_n, Reference data structure ajusted for INS/GNSS measurements.
%   ref_g, Reference data structure ajusted for GNSS measurements.
%
% OUTPUT
%   rmse_v, vector with all RMSE.
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
% Version: 001
% Date:    2018/09/19
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

D2R = (pi/180);     % degrees to radians
R2D = (180/pi);     % radians to degrees

%% INS/GNSS attitude RMSE

RMSE_roll  = rmse (nav_e.roll ,   ref_n.roll)  .* R2D;
RMSE_pitch = rmse (nav_e.pitch,   ref_n.pitch) .* R2D;

% Avoid difference greater than 150 deg when comparing yaw angles.
idx = ( abs(nav_e.yaw - ref_n.yaw) < (150 * D2R) );
RMSE_yaw = rmse (nav_e.yaw(idx),ref_n.yaw(idx) ) .* R2D;

%% INS/GNSS velocity RMSE

if (isfield(nav_e, 'vel') & isfield( ref_n, 'vel'))
    RMSE_vn = rmse (nav_e.vel(:,1),  ref_n.vel(:,1));
    RMSE_ve = rmse (nav_e.vel(:,2),  ref_n.vel(:,2));
    RMSE_vd = rmse (nav_e.vel(:,3),  ref_n.vel(:,3));
else
    RMSE_vn = NaN;
    RMSE_ve = NaN;
    RMSE_vd = NaN;
end

%% INS/GNSS position RMSE

[RM,RN] = radius(nav_e.lat(1), 'double');
LAT2M = (RM + double(nav_e.h(1)));                          % Coefficient for lat rad -> meters
LON2M = (RN + double(nav_e.h(1))) .* cos(nav_e.lat(1));   % Coefficient for lon rad -> meters

RMSE_lat = rmse (nav_e.lat, ref_n.lat) .* LAT2M;
RMSE_lon = rmse (nav_e.lon, ref_n.lon) .* LON2M;
RMSE_h   = rmse (nav_e.h,   ref_n.h);

%% GNSS velocity RMSE

if (isfield(gnss, 'vel') & isfield( ref_g, 'vel'))
    RMSE_vn_g = rmse (gnss.vel(:,1), ref_g.vel(:,1));
    RMSE_ve_g = rmse (gnss.vel(:,2), ref_g.vel(:,2));
    RMSE_vd_g = rmse (gnss.vel(:,3), ref_g.vel(:,3));
else
    RMSE_vn_g = NaN;
    RMSE_ve_g = NaN;
    RMSE_vd_g = NaN;
end

%% GNSS position RMSE

[RM,RN] = radius(gnss.lat(1), 'double');
LAT2M = (RM + double(gnss.h(1)));                        % Coefficient for lat rad -> meters
LON2M = (RN + double(gnss.h(1))) .* cos(gnss.lat(1));     % Coefficient for lon rad -> meters

RMSE_lat_g = rmse (gnss.lat, ref_g.lat) .* LAT2M;
RMSE_lon_g = rmse (gnss.lon, ref_g.lon) .* LON2M;
RMSE_h_g   = rmse (gnss.h, ref_g.h);

rmse_v = [  RMSE_roll;  RMSE_pitch; RMSE_yaw;    
            RMSE_vn;    RMSE_ve;    RMSE_vd;
            RMSE_lat;   RMSE_lon;   RMSE_h;
            RMSE_vn_g;  RMSE_ve_g;  RMSE_vd_g;
            RMSE_lat_g; RMSE_lon_g; RMSE_h_g; ];
end
        