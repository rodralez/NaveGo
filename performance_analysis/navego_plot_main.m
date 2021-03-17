function  navego_plot_main (ref, gnss, nav_e, gnss_i, nav_i, ref_g, ref_n)
% navego_plot: plots results from INS/GNSS integration dataset.
%
% INPUT
%   ref,    reference dataset.
%   gnss,   GNSS dataset.
%   nav_e,  INS/GNSS integration dataset.
%   gnss_i, GNSS dataset interpolated by reference time vector.
%   nav_i,  INS/GNSS dataset interpolated by reference time vector.
%   ref_n,  reference dataset adjusted by INS/GNSS interpolation.
%   ref_g,  reference dataset interpolated by GNSS interpolation.
%
% OUTPUT
%   Several figures.
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
% Version: 013
% Date:    2021/03/15
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

R2D = (180/pi);     % radians to degrees

% Colors
blue    = [0, 0.4470, 0.7410];
% orange  = [0.8500, 0.3250, 0.0980];
% green   = [0.4660, 0.6740, 0.1880];
% yellow  = [0.9290, 0.6940, 0.1250];
% light_blue = [0.3010, 0.7450, 0.9330];

% Text size
font_title  = 50;
font_tick   = 15;
font_label  = 20;
font_legend = 15;

% Line width
lw = 2;

% Standard deviation * 3 vector from Kalman filter a posteriori covariance matrix
sig3_v = abs(nav_e.Pp(:, 1:16:end).^(0.5)) .* 3; %  Taking only diagonal elements from Pp

% 3D TRAJECTORY
figure;
plot3(ref_n.lon.*R2D, ref_n.lat.*R2D, ref_n.h, '--k');
hold on
plot3(nav_i.lon.*R2D, nav_i.lat.*R2D, nav_i.h, '-o', 'Color', blue, 'LineWidth', lw);
plot3(ref_n.lon(1).*R2D, ref_n.lat(1).*R2D, ref_n.h(1), 'or', 'MarkerSize', 10, 'LineWidth', lw);
hold off
axis tight
t1 = title('3D TRAJECTORY');
x1 = xlabel('Longitude [deg]');
y1 = ylabel('Latitude [deg]');
z1 = zlabel('Altitude [m]');
view(45, 45)
l1 = legend('REF', 'INS/GNSS', 'Starting point', 'Location', 'NorthEast');
grid

set(t1,'FontSize', font_title);
set(x1,'FontSize', font_label);
set(y1,'FontSize', font_label);
set(z1,'FontSize', font_label);
set(l1,'FontSize', font_legend);
set(gca, 'YTickMode', 'auto', 'FontSize', font_tick);

% 2D TRAJECTORY
figure;
plot(ref.lon.*R2D, ref.lat.*R2D, '--k');
hold on
plot(nav_i.lon.*R2D, nav_i.lat.*R2D, 'Color', blue, 'LineWidth', lw);
plot(ref.lon(1).*R2D, ref.lat(1).*R2D, 'or', 'MarkerSize', 10, 'LineWidth', lw)
% axis tight
t1 = title('2D TRAJECTORY');
x1 = xlabel('Longitude [deg]');
y1 = ylabel('Latitude [deg]');

l1 = legend('REF', 'INS/GNSS', 'Starting point', 'Location', 'NorthEast');
grid

set(t1,'FontSize', font_title);
set(x1,'FontSize', font_label);
set(y1,'FontSize', font_label);
set(z1,'FontSize', font_label);
set(l1,'FontSize', font_legend);
set(gca, 'YTickMode', 'auto', 'FontSize', font_tick);

% ATTITUDE
figure;

subplot(311)
cell_s = {'ROLL', 'Time [s]', '[deg]', 'REF', 'INS/GNSS'};
navego_plot(cell_s, 'NORMAL', ref.t,  R2D.*ref.roll, nav_e.t, R2D.*nav_e.roll);

subplot(312)
cell_s = {'PITCH', 'Time [s]', '[deg]', 'REF', 'INS/GNSS'};
navego_plot(cell_s, 'NORMAL', ref.t,  R2D.*ref.pitch, nav_e.t, R2D.*nav_e.pitch);

subplot(313)
cell_s = {'YAW', 'Time [s]', '[deg]', 'REF', 'INS/GNSS'};
navego_plot(cell_s, 'NORMAL', ref.t,  R2D.*ref.yaw, nav_e.t, R2D.*nav_e.yaw);

% ATTITUDE ERRORS
figure;

subplot(311)
cell_s = {'ROLL ERROR', 'Time [s]', '[deg]', '3\sigma', 'INS/GNSS'};
navego_plot(cell_s, 'ERROR', nav_e.tg, R2D.*sig3_v(:,1), nav_i.t, R2D.*(nav_i.roll - ref_n.roll));

subplot(312)
cell_s = {'PITCH ERROR', 'Time [s]', '[deg]', '3\sigma', 'INS/GNSS'};
navego_plot(cell_s, 'ERROR', nav_e.tg, R2D.*sig3_v(:,2), nav_i.t, R2D.*(nav_i.pitch - ref_n.pitch));

subplot(313)
cell_s = {'YAW ERROR', 'Time [s]', '[deg]', '3\sigma', 'INS/GNSS'};
navego_plot(cell_s, 'ERROR', nav_e.tg, R2D.*sig3_v(:,3), nav_i.t, R2D.*(nav_i.yaw - ref_n.yaw));

% VELOCITIES
figure;

subplot(311)
cell_s = {'NORTH VELOCITY', 'Time [s]', '[m/s]', 'REF', 'GNSS', 'INS/GNSS'};
navego_plot(cell_s, 'NORMAL', ref.t, ref.vel(:,1), nav_e.t, nav_e.vel(:,1), gnss.t, gnss.vel(:,1));

subplot(312)
cell_s = {'EAST VELOCITY', 'Time [s]', '[m/s]', 'REF', 'GNSS', 'INS/GNSS'};
navego_plot(cell_s, 'NORMAL', ref.t, ref.vel(:,2), nav_e.t, nav_e.vel(:,2), gnss.t, gnss.vel(:,2));

subplot(313)
cell_s = {'DOWN VELOCITY', 'Time [s]', '[m/s]', 'REF', 'GNSS', 'INS/GNSS'};
navego_plot(cell_s, 'NORMAL', ref.t, ref.vel(:,3), nav_e.t, nav_e.vel(:,3), gnss.t, gnss.vel(:,3));

% VELOCITIES ERRORS
figure;

subplot(311)
cell_s = {'VELOCITY NORTH ERROR', 'Time [s]', '[m/s]', '3\sigma', 'GNSS', 'INS/GNSS'};
navego_plot(cell_s, 'ERROR', nav_e.tg, sig3_v(:,4), ...
    nav_i.t, (nav_i.vel(:,1) - ref_n.vel(:,1)), ...
    gnss_i.t, (gnss_i.vel(:,1) - ref_g.vel(:,1)) ) ;
    
subplot(312)
cell_s = {'VELOCITY EAST ERROR', 'Time [s]', '[m/s]', '3\sigma', 'GNSS', 'INS/GNSS'};
navego_plot(cell_s, 'ERROR', nav_e.tg, sig3_v(:,5), ...
    nav_i.t, (nav_i.vel(:,2) - ref_n.vel(:,2)), ...
    gnss_i.t, (gnss_i.vel(:,2) - ref_g.vel(:,2)) ) ;

subplot(313)
cell_s = {'VELOCITY DOWN ERROR', 'Time [s]', '[m/s]', '3\sigma', 'GNSS', 'INS/GNSS'};
navego_plot(cell_s, 'ERROR', nav_e.tg, sig3_v(:,6), ...
        nav_i.t, (nav_i.vel(:,3) - ref_n.vel(:,3)), ...
    gnss_i.t, (gnss_i.vel(:,3) - ref_g.vel(:,3)) ) ;

% POSITION
figure;

subplot(311)
cell_s = {'LATITUDE', 'Time [s]', '[deg]', 'REF', 'GNSS', 'INS/GNSS'};
navego_plot(cell_s, 'NORMAL', ref.t, R2D.*ref.lat, nav_e.t, R2D.*nav_e.lat, gnss.t, R2D.*gnss.lat);

subplot(312)
cell_s = {'LONGITUDE', 'Time [s]', '[deg]', 'REF', 'GNSS', 'INS/GNSS'};
navego_plot(cell_s, 'NORMAL', ref.t, R2D.*ref.lon, nav_e.t, R2D.*nav_e.lon, gnss.t, R2D.*gnss.lon);

subplot(313)
cell_s = {'ALTITUDE', 'Time [s]', '[deg]', 'REF', 'GNSS', 'INS/GNSS'};
navego_plot(cell_s, 'NORMAL', ref.t, ref.h, nav_e.t, nav_e.h, gnss.t, gnss.h);

% POSITION ERRORS
[RN,RE]  = radius(nav_i.lat);
LAT2M_N = RN + nav_i.h;                     % Radians to meters
LON2M_N = (RE + nav_i.h).*cos(nav_i.lat);   % Radians to meters

[RN,RE]  = radius(gnss.lat);
LAT2M_G = RN + gnss.h;                      % Radians to meters
LON2M_G = (RE + gnss.h).*cos(gnss.lat);     % Radians to meters

[RN,RE]  = radius(gnss_i.lat);
LAT2M_I = RN + gnss_i.h;                    % Radians to meters
LON2M_I = (RE + gnss_i.h).*cos(gnss_i.lat); % Radians to meters

figure;

subplot(311)
cell_s = {'LATITUDE ERROR', 'Time [s]', '[m]', '3\sigma', 'GNSS', 'INS/GNSS'};
navego_plot(cell_s, 'ERROR', nav_e.tg, LAT2M_G .* sig3_v(:,7), ...
                             nav_i.t,  LAT2M_N .* (nav_i.lat - ref_n.lat), ... 
                             gnss_i.t, LAT2M_I .* (gnss_i.lat - ref_g.lat) );

subplot(312)
cell_s = {'LONGITUDE ERROR', 'Time [s]', '[m]', '3\sigma', 'GNSS', 'INS/GNSS'};
navego_plot(cell_s, 'ERROR', nav_e.tg, LON2M_G .* sig3_v(:,8), ...
                             nav_i.t,  LON2M_N .* (nav_i.lon - ref_n.lon) , ...
                             gnss_i.t, LON2M_I .* (gnss_i.lon - ref_g.lon) );

subplot(313)
cell_s = {'ALTITUDE ERROR', 'Time [s]', '[m]', '3\sigma', 'GNSS', 'INS/GNSS'};
navego_plot(cell_s, 'ERROR', nav_e.tg, sig3_v(:,9), ...
                             nav_i.t, (nav_i.h - ref_n.h) , ...
                             gnss_i.t, (gnss_i.h - ref_g.h) );

% BIAS ESTIMATION
figure;

subplot(311)
cell_s = {'KF BIAS GYRO X ESTIMATION', 'Time [s]', '[deg]', '3\sigma', 'BIAS'};
navego_plot(cell_s, 'ERROR', nav_e.tg, R2D.*sig3_v(:,10), nav_e.tg, R2D.*nav_e.b(:, 1));

subplot(312)
cell_s = {'KF BIAS GYRO Y ESTIMATION', 'Time [s]', '[deg]', '3\sigma', 'BIAS'};
navego_plot(cell_s, 'ERROR', nav_e.tg, R2D.*sig3_v(:,11), nav_e.tg, R2D.*nav_e.b(:, 2));

subplot(313)
cell_s = {'KF BIAS GYRO Z ESTIMATION', 'Time [s]', '[deg]', '3\sigma', 'BIAS'};
navego_plot(cell_s, 'ERROR', nav_e.tg, R2D.*sig3_v(:,12), nav_e.tg, R2D.*nav_e.b(:, 3));

figure;

subplot(311)
cell_s = {'KF BIAS ACCR X ESTIMATION', 'Time [s]', '[m/s]', '3\sigma', 'BIAS'};
navego_plot(cell_s, 'ERROR', nav_e.tg, sig3_v(:,13), nav_e.tg, nav_e.b(:, 4));

subplot(312)
cell_s = {'KF BIAS ACCR Y ESTIMATION', 'Time [s]', '[m/s]', '3\sigma', 'BIAS'};
navego_plot(cell_s, 'ERROR', nav_e.tg, sig3_v(:,14), nav_e.tg, nav_e.b(:, 5));

subplot(313)
cell_s = {'KF BIAS ACCR Z ESTIMATION', 'Time [s]', '[m/s]', '3\sigma', 'BIAS'};
navego_plot(cell_s, 'ERROR', nav_e.tg, sig3_v(:,15), nav_e.tg, nav_e.b(:, 6));
