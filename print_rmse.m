function [imu_r, gps_r] = print_rmse (imu, gps, ref, string)
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
% Version: 002
% Date:    2016/10/25
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego 

global R2D

gps_r.t = gps.t;
gps_r.lat = interp1(ref.t, ref.lat, gps.t, 'linear');
gps_r.lon = interp1(ref.t, ref.lon, gps.t, 'linear');
gps_r.h   = interp1(ref.t, ref.h,   gps.t, 'linear');
gps_r.vel = interp1(ref.t, ref.vel, gps.t, 'linear');

if( isnan(gps_r.lat))
    error('ERROR in print_rmse: NaN elements in GPS reference vector')
end

imu_r.t = imu.t;
imu_r.roll  = interp1(ref.t, ref.roll,  imu.t, 'linear');
imu_r.pitch = interp1(ref.t, ref.pitch, imu.t, 'linear');
imu_r.yaw   = interp1(ref.t, ref.yaw,   imu.t, 'linear');
imu_r.lat = interp1(ref.t, ref.lat, imu.t, 'linear');
imu_r.lon = interp1(ref.t, ref.lon, imu.t, 'linear');
imu_r.h   = interp1(ref.t, ref.h,   imu.t, 'linear');
imu_r.vel = interp1(ref.t, ref.vel, imu.t, 'linear');

if( isnan(imu_r.lat))
    error('ERROR in print_rmse: NaN elements in IMU reference vector')
end

[RN,RE] = radius(imu.lat(1), 'double');
LAT2M = (RN + double(imu.h(1)));                        % Latitude to meters constant                       
LON2M = (RE + double(imu.h(1))) .* cos(imu.lat(1));   % Longitude to meters constant

% INS/GPS attitude RMSE
RMSE_roll   = rmse (imu.roll ,  imu_r.roll)  .* R2D;
RMSE_pitch  = rmse (imu.pitch,  imu_r.pitch) .* R2D;
% RMSE_yaw    = rmse (imu1_e.yaw,   imu_r.yaw).*r2d;
% Only compare those estimates that have a diff. < pi with respect to ref
idx = find ( abs(imu.yaw - imu_r.yaw) < pi );
RMSE_yaw    = rmse (imu.yaw(idx),   imu_r.yaw(idx)).* R2D;

% INS/GPS velocity RMSE
RMSE_vn     = rmse (imu.vel(:,1),  imu_r.vel(:,1));
RMSE_ve     = rmse (imu.vel(:,2),  imu_r.vel(:,2));
RMSE_vd     = rmse (imu.vel(:,3),  imu_r.vel(:,3));

% INS/GPS position RMSE
RMSE_lat    = rmse (imu.lat, imu_r.lat) .* LAT2M;
RMSE_lon    = rmse (imu.lon, imu_r.lon) .* LON2M;
RMSE_h      = rmse (imu.h,         imu_r.h);

[RN,RE] = radius(gps.lat(1), 'double');
LAT2M = (RN + double(gps.h(1)));                        % Latitude to meters constant 
LON2M = (RE + double(gps.h(1))) .* cos(gps.lat(1));     % Longitude to meters constant

% GPS RMSE
RMSE_lat_g  = rmse (gps.lat, gps_r.lat) .* LAT2M;
RMSE_lon_g  = rmse (gps.lon, gps_r.lon) .* LON2M;
RMSE_h_g    = rmse (gps.h-gps.larm(3), gps_r.h);
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