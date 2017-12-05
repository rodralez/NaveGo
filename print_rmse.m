function rmse_v = print_rmse (ins_gps, gps, ins_gps_r, gps_r, string)
% print_rmse: print on console Root Mean Squared Errors (RMSE) between INS/GPS
% and a reference, and between GPS-only and a reference as well.
%
% INPUT:
%   ins_gps, INS/GPS data structure.
%   gps, GPS data structure.
%   ins_gps_r, Reference data structure ajusted for INS/GPS measurements.
%   gps_r, Reference data structure ajusted for GPS measurements.
%   string, string to print on console identifying the RMSE.
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
% Version: 005
% Date:    2017/05/19
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

D2R = (pi/180);     % degrees to radians
R2D = (180/pi);     % radians to degrees

%% INS/GPS attitude RMSE

RMSE_roll  = rmse (ins_gps.roll ,   ins_gps_r.roll)  .* R2D;
RMSE_pitch = rmse (ins_gps.pitch,   ins_gps_r.pitch) .* R2D;

% Avoid difference greater than 175 deg when comparing yaw angles.
idx = ( abs(ins_gps.yaw - ins_gps_r.yaw) < (175 * D2R) );
RMSE_yaw = rmse (ins_gps.yaw(idx),ins_gps_r.yaw(idx) ) .* R2D;

%% INS/GPS velocity RMSE

if (isfield(ins_gps, 'vel') & isfield( ins_gps_r, 'vel'))
    RMSE_vn = rmse (ins_gps.vel(:,1),  ins_gps_r.vel(:,1));
    RMSE_ve = rmse (ins_gps.vel(:,2),  ins_gps_r.vel(:,2));
    RMSE_vd = rmse (ins_gps.vel(:,3),  ins_gps_r.vel(:,3));
else
    RMSE_vn = NaN;
    RMSE_ve = NaN;
    RMSE_vd = NaN;
end

%% INS/GPS position RMSE

[RM,RN] = radius(ins_gps.lat(1), 'double');
LAT2M = (RM + double(ins_gps.h(1)));                          % Coefficient for lat rad -> meters
LON2M = (RN + double(ins_gps.h(1))) .* cos(ins_gps.lat(1));   % Coefficient for lon rad -> meters

RMSE_lat = rmse (ins_gps.lat, ins_gps_r.lat) .* LAT2M;
RMSE_lon = rmse (ins_gps.lon, ins_gps_r.lon) .* LON2M;
RMSE_h   = rmse (ins_gps.h,   ins_gps_r.h);

%% GPS velocity RMSE

if (isfield(gps, 'vel') & isfield( gps_r, 'vel'))
    RMSE_vn_g = rmse (gps.vel(:,1), gps_r.vel(:,1));
    RMSE_ve_g = rmse (gps.vel(:,2), gps_r.vel(:,2));
    RMSE_vd_g = rmse (gps.vel(:,3), gps_r.vel(:,3));
else
    RMSE_vn_g = NaN;
    RMSE_ve_g = NaN;
    RMSE_vd_g = NaN;
end

%% GPS position RMSE

[RM,RN] = radius(gps.lat(1), 'double');
LAT2M = (RM + double(gps.h(1)));                        % Coefficient for lat rad -> meters
LON2M = (RN + double(gps.h(1))) .* cos(gps.lat(1));     % Coefficient for lon rad -> meters

RMSE_lat_g = rmse (gps.lat, gps_r.lat) .* LAT2M;
RMSE_lon_g = rmse (gps.lon, gps_r.lon) .* LON2M;
RMSE_h_g   = rmse (gps.h, gps_r.h);

%% Print RMSE

fprintf( '\n>> RMSE for %s\n', string);

fprintf(' Roll,  %s = %.4e deg \n',   string, RMSE_roll);
fprintf(' Pitch, %s = %.4e deg \n',   string, RMSE_pitch);
fprintf(' Yaw,   %s = %.4e deg \n\n', string, RMSE_yaw);

if (isfield(ins_gps, 'vel') & isfield( ins_gps_r, 'vel') & isfield(gps, 'vel') & isfield( gps_r, 'vel'))
    fprintf(' Vel. N, %s = %.4e m/s, GPS = %.4e m/s \n',   string, RMSE_vn, RMSE_vn_g);
    fprintf(' Vel. E, %s = %.4e m/s, GPS = %.4e m/s \n',   string, RMSE_ve, RMSE_ve_g);
    fprintf(' Vel. D, %s = %.4e m/s, GPS = %.4e m/s \n\n', string, RMSE_vd, RMSE_vd_g);
end

fprintf(' Latitude,  %s = %.4e m, GPS = %.4e m \n', string, RMSE_lat, RMSE_lat_g);
fprintf(' Longitude, %s = %.4e m, GPS = %.4e m \n', string, RMSE_lon, RMSE_lon_g);
fprintf(' Altitude,  %s = %.4e m, GPS = %.4e m \n', string, RMSE_h,   RMSE_h_g);

%% Output data structure

rmse_v = [  RMSE_roll;  RMSE_pitch; RMSE_yaw;    
            RMSE_vn;    RMSE_ve;    RMSE_vd;
            RMSE_lat;   RMSE_lon;   RMSE_h;
            RMSE_vn_g;  RMSE_ve_g;  RMSE_vd_g;
            RMSE_lat_g; RMSE_lon_g; RMSE_h_g; ];

end
