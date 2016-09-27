function ref_imu = print_rmse (imu_e, ref_imu, gps, ref_g, string)
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
% Version: 001
% Date:    2016/09/27
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego 

global R2D

fe = max(size(imu_e.t));
fr = max(size(ref_imu.t));

% Adjust ref size if it is bigger than estimates size
if (fe < fr)
    
    ref_imu.t     = ref_imu.t(1:fe, :);
    ref_imu.roll  = ref_imu.roll(1:fe, :);
    ref_imu.pitch = ref_imu.pitch(1:fe, :);
    ref_imu.yaw   = ref_imu.yaw(1:fe, :);
    ref_imu.vel   = ref_imu.vel(1:fe, :);
    ref_imu.lat   = ref_imu.lat(1:fe, :);
    ref_imu.lon   = ref_imu.lon(1:fe, :);
    ref_imu.h     = ref_imu.h(1:fe, :);
    ref_imu.DCMnb = ref_imu.DCMnb(1:fe, :);
end

[RN,RE] = radius(imu_e.lat(1), 'double');
LAT2M = (RN + double(imu_e.h(1)));                        % Latitude to meters contant                       
LON2M = (RE + double(imu_e.h(1))) .* cos(imu_e.lat(1));   % Longitude to meters contant

% INS/GPS attitude RMSE
RMSE_roll   = rmse (imu_e.roll ,  ref_imu.roll)  .* R2D;
RMSE_pitch  = rmse (imu_e.pitch,  ref_imu.pitch) .* R2D;
% RMSE_yaw    = rmse (imu1_e.yaw,   ref_imu.yaw).*r2d;
% Only compare those estimates that have a diff. < pi with respect to ref
idx = find ( abs(imu_e.yaw - ref_imu.yaw) < pi );
RMSE_yaw    = rmse (imu_e.yaw(idx),   ref_imu.yaw(idx)).* R2D;

% INS/GPS velocity RMSE
RMSE_vn     = rmse (imu_e.vel(:,1),  ref_imu.vel(:,1));
RMSE_ve     = rmse (imu_e.vel(:,2),  ref_imu.vel(:,2));
RMSE_vd     = rmse (imu_e.vel(:,3),  ref_imu.vel(:,3));

% INS/GPS position RMSE
RMSE_lat    = rmse (imu_e.lat, ref_imu.lat) .* LAT2M;
RMSE_lon    = rmse (imu_e.lon, ref_imu.lon) .* LON2M;
RMSE_h      = rmse (imu_e.h,         ref_imu.h);

[RN,RE] = radius(gps.lat(1), 'double');
LAT2M = (RN + double(gps.h(1)));                        % Latitude to meters contant 
LON2M = (RE + double(gps.h(1))) .* cos(gps.lat(1));     % Longitude to meters contant

% GPS RMSE
RMSE_lat_g  = rmse (gps.lat, ref_g.lat) .* LAT2M;
RMSE_lon_g  = rmse (gps.lon, ref_g.lon) .* LON2M;
RMSE_h_g    = rmse (gps.h-gps.larm(3), ref_g.h);
RMSE_vn_g   = rmse (gps.vel(:,1),   ref_g.vel(:,1));
RMSE_ve_g   = rmse (gps.vel(:,2),   ref_g.vel(:,2));
RMSE_vd_g   = rmse (gps.vel(:,3),   ref_g.vel(:,3));

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