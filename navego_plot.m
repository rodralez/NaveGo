function  navego_plot (ref, gps, nav_e, gps_ref, nav_ref, ref_n, ref_g)
% navego_plot: 
%
% INPUT:
%   ref, INS/GPS data structure.
%   gps, GPS data structure.
%   nav_e, Reference data structure ajusted for navigation measurements.
%   gps_ref, Reference data structure ajusted for GPS measurements.\
%   ref_n, 
%   ref_g
%
% OUTPUT
%   several figures.
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

sig3_rr = abs(nav_e.Pp(:, 1:22:end).^(0.5)) .* 3; % Only take diagonal elements from Pp

% TRAJECTORY
figure;
plot3(ref.lon.*R2D, ref.lat.*R2D, ref.h, '--k')
hold on
%     plot3(nav_e.lon.*R2D, nav_e.lat.*R2D, nav_e.h, '-b')
plot3(ref.lon(1).*R2D, ref.lat(1).*R2D, ref.h(1), 'or', 'MarkerSize', 10, 'LineWidth', 2)
axis tight
title('TRAJECTORY')
xlabel('Longitude [deg.]')
ylabel('Latitude [deg.]')
zlabel('Altitude [m]')
view(0, 90)
grid

% ATTITUDE
figure;
subplot(311)
plot(ref.t, R2D.*ref.roll, '--k', nav_e.t, R2D.*nav_e.roll,'-b');
ylabel('[deg]')
xlabel('Time [s]')
legend('REF', 'NAV_E');
title('ROLL');
grid

subplot(312)
plot(ref.t, R2D.*ref.pitch, '--k', nav_e.t, R2D.*nav_e.pitch,'-b');
ylabel('[deg]')
xlabel('Time [s]')
legend('REF', 'NAV_E')
title('PITCH');
grid

subplot(313)
plot(ref.t, R2D.* ref.yaw, '--k', nav_e.t, R2D.*nav_e.yaw,'-b');
ylabel('[deg]')
xlabel('Time [s]')
legend('REF', 'NAV_E')
title('YAW');
grid

% ATTITUDE ERRORS
figure;
subplot(311)
plot(nav_e.t, (nav_ref.roll - ref_n.roll).*R2D, '-b' );
hold on
plot (nav_e.tk, R2D.*sig3_rr(:,1), '--k', nav_e.tk, -R2D.*sig3_rr(:,1), '--k' )
ylabel('[deg]')
xlabel('Time [s]')
legend('NAV_E', '3\sigma');
title('ROLL ERROR');
grid

subplot(312)
plot(nav_e.t, (nav_ref.pitch - ref_n.pitch).*R2D, '-b' );
hold on
plot (nav_e.tk, R2D.*sig3_rr(:,2), '--k', nav_e.tk, -R2D.*sig3_rr(:,2), '--k' )
ylabel('[deg]')
xlabel('Time [s]')
legend('NAV_E', '3\sigma');
title('PITCH ERROR');
grid

subplot(313)
plot(nav_e.t, (nav_ref.yaw - ref_n.yaw).*R2D, '-b' );
hold on
plot (nav_e.tk, R2D.*sig3_rr(:,3), '--k', nav_e.tk, -R2D.*sig3_rr(:,3), '--k' )
ylabel('[deg]')
xlabel('Time [s]')
legend('NAV_E', '3\sigma');
title('YAW ERROR');
grid

% VELOCITIES
figure;
subplot(311)
plot(ref.t, ref.vel(:,1), '--k', gps.t, gps.vel(:,1),'-c', nav_e.t, nav_e.vel(:,1),'-b' );
xlabel('Time [s]')
ylabel('[m/s]')
legend('REF', 'GPS', 'NAV_E');
title('NORTH VELOCITY');
grid

subplot(312)
plot(ref.t, ref.vel(:,2), '--k', gps.t, gps.vel(:,2),'-c', nav_e.t, nav_e.vel(:,2),'-b' );
xlabel('Time [s]')
ylabel('[m/s]')
legend('REF', 'GPS', 'NAV_E');
title('EAST VELOCITY');
grid

subplot(313)
plot(ref.t, ref.vel(:,3), '--k', gps.t, gps.vel(:,3),'-c', nav_e.t, nav_e.vel(:,3),'-b' );
xlabel('Time [s]')
ylabel('[m/s]')
legend('REF', 'GPS', 'NAV_E');
title('DOWN VELOCITY');
grid

% VELOCITIES ERRORS
figure;
subplot(311)
plot(gps_ref.t, (gps_ref.vel(:,1) - ref_g.vel(:,1)), '-c');
hold on
plot(nav_ref.t, (nav_ref.vel(:,1) - ref_n.vel(:,1)), '-b' );
plot (nav_e.tk, sig3_rr(:,4), '--k', nav_e.tk, -sig3_rr(:,4), '--k' )
xlabel('Time [s]')
ylabel('[m/s]')
legend('GPS', 'NAV_E', '3\sigma');
title('VELOCITY NORTH ERROR');
grid

subplot(312)
plot(gps_ref.t, (gps_ref.vel(:,2) - ref_g.vel(:,2)), '-c');
hold on
plot(nav_ref.t, (nav_ref.vel(:,2) - ref_n.vel(:,2)), '-b' );
plot (nav_e.tk, sig3_rr(:,5), '--k', nav_e.tk, -sig3_rr(:,5), '--k' )
xlabel('Time [s]')
ylabel('[m/s]')
legend('GPS', 'NAV_E', '3\sigma');
title('VELOCITY EAST ERROR');
grid

subplot(313)
plot(gps_ref.t, (gps_ref.vel(:,3) - ref_g.vel(:,3)), '-c');
hold on
plot(nav_ref.t, (nav_ref.vel(:,3) - ref_n.vel(:,3)), '-b' );
plot (nav_e.tk, sig3_rr(:,6), '--k', nav_e.tk, -sig3_rr(:,6), '--k' )
xlabel('Time [s]')
ylabel('[m/s]')
legend('GPS', 'NAV_E', '3\sigma');
title('VELOCITY DOWN ERROR');
grid

% POSITION
figure;
subplot(311)
plot(ref.t, ref.lat .*R2D, '--k', gps.t, gps.lat.*R2D, '-c', nav_e.t, nav_e.lat.*R2D, '-b');
xlabel('Time [s]')
ylabel('[deg]')
legend('REF', 'GPS', 'NAV_E' );
title('LATITUDE');
grid

subplot(312)
plot(ref.t, ref.lon .*R2D, '--k', gps.t, gps.lon.*R2D, '-c', nav_e.t, nav_e.lon.*R2D, '-b');
xlabel('Time [s]')
ylabel('[deg]')
legend('REF', 'GPS', 'NAV_E' );
title('LONGITUDE');
grid

subplot(313)
plot(ref.t, ref.h, '--k', gps.t, gps.h, '-c', nav_e.t, nav_e.h, '-b')
xlabel('Time [s]')
ylabel('[m]')
legend('REF', 'GPS', 'NAV_E');
title('ALTITUDE');
grid

% POSITION ERRORS
[RN,RE]  = radius(nav_ref.lat, 'double');
LAT2M_1 = RN + nav_ref.h;
LON2M_1 = (RE + nav_ref.h).*cos(nav_ref.lat);

[RN,RE]  = radius(gps.lat, 'double');
LAT2M_G = RN + gps.h;
LON2M_G = (RE + gps.h).*cos(gps.lat);

[RN,RE]  = radius(gps_ref.lat, 'double');
LAT2M_GR = RN + gps_ref.h;
LON2M_GR = (RE + gps_ref.h).*cos(gps_ref.lat);

figure;
subplot(311)
plot(gps_ref.t,  LAT2M_GR.*(gps_ref.lat - ref_g.lat), '-c')
hold on
plot(nav_ref.t, LAT2M_1.*(nav_ref.lat - ref_n.lat), '-b')
plot (nav_e.tk, LAT2M_G.*sig3_rr(:,7), '--k', nav_e.tk, -LAT2M_G.*sig3_rr(:,7), '--k' )
xlabel('Time [s]')
ylabel('[m]')
legend('GPS', 'NAV_E', '3\sigma');
title('LATITUDE ERROR');
grid

subplot(312)
plot(gps_ref.t, LON2M_GR.*(gps_ref.lon - ref_g.lon), '-c')
hold on
plot(nav_ref.t, LON2M_1.*(nav_ref.lon - ref_n.lon), '-b')
plot(nav_e.tk, LON2M_G.*sig3_rr(:,8), '--k', nav_e.tk, -LON2M_G.*sig3_rr(:,8), '--k' )
xlabel('Time [s]')
ylabel('[m]')
legend('GPS', 'NAV_E', '3\sigma');
title('LONGITUDE ERROR');
grid

subplot(313)
plot(gps_ref.t, (gps_ref.h - ref_g.h), '-c')
hold on
plot(nav_ref.t, (nav_ref.h - ref_n.h), '-b')
plot(nav_e.tk, sig3_rr(:,9), '--k', nav_e.tk, -sig3_rr(:,9), '--k' )
xlabel('Time [s]')
ylabel('[m]')
legend('GPS', 'NAV_E', '3\sigma');
title('ALTITUDE ERROR');
grid
