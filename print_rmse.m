function print_rmse (ins_gps, gps, ins_gps_r, gps_r, string)
% print_rmse: print on console Root Mean Squared Errors between INS/GPS
% and reference, and GPS-only and reference as well.
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
% Version: 003
% Date:    2016/11/17
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego 

global R2D

[RN,RE] = radius(ins_gps.lat(1), 'double');
LAT2M = (RN + double(ins_gps.h(1)));                          % Coefficient for lat rad -> meters               
LON2M = (RE + double(ins_gps.h(1))) .* cos(ins_gps.lat(1));   % Coefficient for lon rad -> meters

% INS/GPS attitude RMSE
RMSE_roll  = rmse (ins_gps.roll ,   ins_gps_r.roll)  .* R2D;
RMSE_pitch = rmse (ins_gps.pitch,   ins_gps_r.pitch) .* R2D;
% Only compare those estimates that have a diff. < pi with respect to ref
idx = find ( abs(ins_gps.yaw - ins_gps_r.yaw) < pi );
RMSE_yaw   = rmse (ins_gps.yaw(idx),ins_gps_r.yaw(idx)).* R2D;

% INS/GPS velocity RMSE
RMSE_vn     = rmse (ins_gps.vel(:,1),  ins_gps_r.vel(:,1));
RMSE_ve     = rmse (ins_gps.vel(:,2),  ins_gps_r.vel(:,2));
RMSE_vd     = rmse (ins_gps.vel(:,3),  ins_gps_r.vel(:,3));

% INS/GPS position RMSE
RMSE_lat    = rmse (ins_gps.lat, ins_gps_r.lat) .* LAT2M;
RMSE_lon    = rmse (ins_gps.lon, ins_gps_r.lon) .* LON2M;
RMSE_h      = rmse (ins_gps.h,         ins_gps_r.h);

[RN,RE] = radius(gps.lat(1), 'double');
LAT2M = (RN + double(gps.h(1)));                        % Coefficient for lat rad -> meters
LON2M = (RE + double(gps.h(1))) .* cos(gps.lat(1));     % Coefficient for lon rad -> meters

% GPS RMSE
RMSE_lat_g  = rmse (gps.lat, gps_r.lat) .* LAT2M;
RMSE_lon_g  = rmse (gps.lon, gps_r.lon) .* LON2M;
RMSE_h_g    = rmse (gps.h, gps_r.h);
RMSE_vn_g   = rmse (gps.vel(:,1),   gps_r.vel(:,1));
RMSE_ve_g   = rmse (gps.vel(:,2),   gps_r.vel(:,2));
RMSE_vd_g   = rmse (gps.vel(:,3),   gps_r.vel(:,3));

% Print RMSE
fprintf( '\n>> RMSE for %s\n', string);

fprintf( ' Roll,  %s = %.4e deg\n', string, RMSE_roll);
fprintf( ' Pitch, %s = %.4e deg\n', string, RMSE_pitch);
fprintf( ' Yaw,   %s = %.4e deg\n\n', string, RMSE_yaw);

fprintf( ' Vel. N, %s = %.4e m/s, GPS = %.4e m/s\n', string, RMSE_vn, RMSE_vn_g);
fprintf( ' Vel. E, %s = %.4e m/s, GPS = %.4e m/s\n', string, RMSE_ve, RMSE_ve_g);
fprintf( ' Vel. D, %s = %.4e m/s, GPS = %.4e m/s\n\n', string, RMSE_vd, RMSE_vd_g);

fprintf( ' Latitude,  %s = %.4e m, GPS = %.4e m\n', string, RMSE_lat, RMSE_lat_g);
fprintf( ' Longitude, %s = %.4e m, GPS = %.4e m\n', string, RMSE_lon, RMSE_lon_g);
fprintf( ' Altitude,  %s = %.4e m, GPS = %.4e m\n', string, RMSE_h, RMSE_h_g);

end
