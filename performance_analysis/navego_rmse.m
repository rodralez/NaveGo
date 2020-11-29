function rmse_v = navego_rmse (nav, gnss, ref_n, ref_g)
% navego_rmse: calculates the Root Mean Squared Errors (RMSE) between 
% a INS/GNSS system and a reference data structure, and between GNSS-only 
% solution and a reference data structure.
%
% INPUT
%   nav_e, INS/GNSS integration data structure.
%   gnss,  GNSS data structure.
%   ref_n, Reference data structure ajusted for INS/GNSS estimations.
%   ref_g, Reference data structure ajusted for GNSS measurements.
%
% OUTPUT
%   rmse_v, vector with all RMSE.
%       RMSE_roll;  RMSE_pitch; RMSE_yaw; (degrees, degrees, degrees)    
%       RMSE_vn;    RMSE_ve;    RMSE_vd;  (m/s, m/s, m/s) 
%       RMSE_lat;   RMSE_lon;   RMSE_h;   (m, m, m)
%       RMSE_vn_g;  RMSE_ve_g;  RMSE_vd_g;(m/s, m/s, m/s)
%       RMSE_lat_g; RMSE_lon_g; RMSE_h_g; (m, m, m)
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
% Version: 005
% Date:    2019/04/05
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

D2R = (pi/180);     % degrees to radians
R2D = (180/pi);     % radians to degrees

%% INS/GNSS attitude RMSE

RMSE_roll  = rmse (nav.roll , ref_n.roll)  .* R2D;
RMSE_pitch = rmse (nav.pitch, ref_n.pitch) .* R2D;

% Avoid difference greater than 100 deg when comparing yaw angles.
idx = ( abs(nav.yaw - ref_n.yaw) < (100 * D2R) );
RMSE_yaw = rmse ( nav.yaw(idx),ref_n.yaw(idx) ) .* R2D;

%% INS/GNSS velocity RMSE

if (isfield(nav, 'vel') && isfield( ref_n, 'vel'))
    RMSE_vn = rmse (nav.vel(:,1),  ref_n.vel(:,1));
    RMSE_ve = rmse (nav.vel(:,2),  ref_n.vel(:,2));
    RMSE_vd = rmse (nav.vel(:,3),  ref_n.vel(:,3));
else
    RMSE_vn = NaN;
    RMSE_ve = NaN;
    RMSE_vd = NaN;
end

%% INS/GNSS position RMSE

[RM,RN] = radius(ref_n.lat);
LAT2M = (RM + ref_n.h);                     % Coefficient for lat, radians to meters
LON2M = (RN + ref_n.h) .* cos(ref_n.lat);   % Coefficient for lon, radians to meters

RMSE_lat = rmse (nav.lat.* LAT2M, ref_n.lat.* LAT2M) ;
RMSE_lon = rmse (nav.lon.* LON2M, ref_n.lon.* LON2M) ;
RMSE_h   = rmse (nav.h,   ref_n.h);

%% GNSS velocity RMSE

if (isfield(gnss, 'vel') && isfield( ref_g, 'vel'))
    RMSE_vn_g = rmse (gnss.vel(:,1), ref_g.vel(:,1));
    RMSE_ve_g = rmse (gnss.vel(:,2), ref_g.vel(:,2));
    RMSE_vd_g = rmse (gnss.vel(:,3), ref_g.vel(:,3));
else
    RMSE_vn_g = NaN;
    RMSE_ve_g = NaN;
    RMSE_vd_g = NaN;
end

%% GNSS position RMSE

[RMg,RNg] = radius(ref_g.lat);
LAT2Mg = (RMg + ref_g.h);                   % Coefficient for lat, radians to meters
LON2Mg = (RNg + ref_g.h) .* cos(ref_g.lat); % Coefficient for lon, radians to meters

RMSE_lat_g = rmse (gnss.lat.* LAT2Mg, ref_g.lat.* LAT2Mg) ;
RMSE_lon_g = rmse (gnss.lon.* LON2Mg, ref_g.lon.* LON2Mg) ;
RMSE_h_g   = rmse (gnss.h, ref_g.h);

rmse_v = [  RMSE_roll;  RMSE_pitch; RMSE_yaw;    
            RMSE_vn;    RMSE_ve;    RMSE_vd;
            RMSE_lat;   RMSE_lon;   RMSE_h;
            RMSE_vn_g;  RMSE_ve_g;  RMSE_vd_g;
            RMSE_lat_g; RMSE_lon_g; RMSE_h_g; ];
end
        