function  navego_plot (ref, gnss, nav_e, gnss_ref, nav_ref, ref_g, ref_n)
% navego_plot: 
%
% INPUT:
%   ref, reference dataset.
%   gnss, GNSS dataset.
%   nav_e, INS/GNSS integration dataset.
%   gnss_ref, GNSS dataset interpolated by reference time vector.
%   nav_ref, INS/GNSS dataset interpolated by reference time vector.
%   ref_n, reference dataset adjusted by interpolation operator.
%   ref_g, referenceS dataset interpolated by interpolation operator.
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
% Version: 006
% Date:    2018/10/16
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

% D2R = (pi/180);     % degrees to radians
R2D = (180/pi);     % radians to degrees

sig3_rr = abs(nav_e.Pp(:, 1:22:end).^(0.5)) .* 3; % Only take diagonal elements from Pp

% TRAJECTORY
figure;
plot3(ref_n.lon.*R2D, ref_n.lat.*R2D, ref_n.h, '--k')
hold on
plot3(nav_ref.lon.*R2D, nav_ref.lat.*R2D, nav_ref.h, '-b')
plot3(ref_n.lon(1).*R2D, ref_n.lat(1).*R2D, ref_n.h(1), 'or', 'MarkerSize', 10, 'LineWidth', 2)
axis tight
title('TRAJECTORY')
xlabel('Longitude [deg]')
ylabel('Latitude [deg]')
zlabel('Altitude [m]')
view(0, 90)
legend('REF', 'INS/GNSS', 'Location','best');
grid

% ATTITUDE
figure;
subplot(311)
plot(ref.t, R2D.*ref.roll, '--k', nav_e.t, R2D.*nav_e.roll,'-b');
ylabel('[deg]')
xlabel('Time [s]')
legend('REF', 'INS/GNSS');
title('ROLL');
grid

subplot(312)
plot(ref.t, R2D.*ref.pitch, '--k', nav_e.t, R2D.*nav_e.pitch,'-b');
ylabel('[deg]')
xlabel('Time [s]')
legend('REF', 'INS/GNSS')
title('PITCH');
grid

subplot(313)
plot(ref.t, R2D.* ref.yaw, '--k', nav_e.t, R2D.*nav_e.yaw,'-b');
ylabel('[deg]')
xlabel('Time [s]')
legend('REF', 'INS/GNSS')
title('YAW');
grid

% ATTITUDE ERRORS
figure;
subplot(311)
plot(nav_ref.t, (nav_ref.roll - ref_n.roll).*R2D, '-b' );
hold on
plot (nav_e.tg, R2D.*sig3_rr(:,1), '--k', nav_e.tg, -R2D.*sig3_rr(:,1), '--k' )
ylabel('[deg]')
xlabel('Time [s]')
legend('INS/GNSS', '3\sigma');
title('ROLL ERROR');
grid

subplot(312)
plot(nav_ref.t, (nav_ref.pitch - ref_n.pitch).*R2D, '-b' );
hold on
plot (nav_e.tg, R2D.*sig3_rr(:,2), '--k', nav_e.tg, -R2D.*sig3_rr(:,2), '--k' )
ylabel('[deg]')
xlabel('Time [s]')
legend('INS/GNSS', '3\sigma');
title('PITCH ERROR');
grid

subplot(313)
plot(nav_ref.t, (nav_ref.yaw - ref_n.yaw).*R2D, '-b' );
hold on
plot (nav_e.tg, R2D.*sig3_rr(:,3), '--k', nav_e.tg, -R2D.*sig3_rr(:,3), '--k' )
ylabel('[deg]')
xlabel('Time [s]')
legend('INS/GNSS', '3\sigma');
title('YAW ERROR');
grid

% VELOCITIES
figure;
subplot(311)
plot(ref.t, ref.vel(:,1), '--k', gnss.t, gnss.vel(:,1),'-c', nav_e.t, nav_e.vel(:,1),'-b' );
xlabel('Time [s]')
ylabel('[m/s]')
legend('REF', 'GNSS', 'INS/GNSS');
title('NORTH VELOCITY');
grid

subplot(312)
plot(ref.t, ref.vel(:,2), '--k', gnss.t, gnss.vel(:,2),'-c', nav_e.t, nav_e.vel(:,2),'-b' );
xlabel('Time [s]')
ylabel('[m/s]')
legend('REF', 'GNSS', 'INS/GNSS');
title('EAST VELOCITY');
grid

subplot(313)
plot(ref.t, ref.vel(:,3), '--k', gnss.t, gnss.vel(:,3),'-c', nav_e.t, nav_e.vel(:,3),'-b' );
xlabel('Time [s]')
ylabel('[m/s]')
legend('REF', 'GNSS', 'INS/GNSS');
title('DOWN VELOCITY');
grid

% VELOCITIES ERRORS
figure;
subplot(311)
plot(gnss_ref.t, (gnss_ref.vel(:,1) - ref_g.vel(:,1)), '-c');
hold on
plot(nav_ref.t, (nav_ref.vel(:,1) - ref_n.vel(:,1)), '-b' );
plot (nav_e.tg, sig3_rr(:,4), '--k', nav_e.tg, -sig3_rr(:,4), '--k' )
xlabel('Time [s]')
ylabel('[m/s]')
legend('GNSS', 'INS/GNSS', '3\sigma');
title('VELOCITY NORTH ERROR');
grid

subplot(312)
plot(gnss_ref.t, (gnss_ref.vel(:,2) - ref_g.vel(:,2)), '-c');
hold on
plot(nav_ref.t, (nav_ref.vel(:,2) - ref_n.vel(:,2)), '-b' );
plot (nav_e.tg, sig3_rr(:,5), '--k', nav_e.tg, -sig3_rr(:,5), '--k' )
xlabel('Time [s]')
ylabel('[m/s]')
legend('GNSS', 'INS/GNSS', '3\sigma');
title('VELOCITY EAST ERROR');
grid

subplot(313)
plot(gnss_ref.t, (gnss_ref.vel(:,3) - ref_g.vel(:,3)), '-c');
hold on
plot(nav_ref.t, (nav_ref.vel(:,3) - ref_n.vel(:,3)), '-b' );
plot (nav_e.tg, sig3_rr(:,6), '--k', nav_e.tg, -sig3_rr(:,6), '--k' )
xlabel('Time [s]')
ylabel('[m/s]')
legend('GNSS', 'INS/GNSS', '3\sigma');
title('VELOCITY DOWN ERROR');
grid

% POSITION
figure;
subplot(311)
plot(ref.t, ref.lat .*R2D, '--k', gnss.t, gnss.lat.*R2D, '-c', nav_e.t, nav_e.lat.*R2D, '-b');
xlabel('Time [s]')
ylabel('[deg]')
legend('REF', 'GNSS', 'INS/GNSS' );
title('LATITUDE');
grid

subplot(312)
plot(ref.t, ref.lon .*R2D, '--k', gnss.t, gnss.lon.*R2D, '-c', nav_e.t, nav_e.lon.*R2D, '-b');
xlabel('Time [s]')
ylabel('[deg]')
legend('REF', 'GNSS', 'INS/GNSS' );
title('LONGITUDE');
grid

subplot(313)
plot(ref.t, ref.h, '--k', gnss.t, gnss.h, '-c', nav_e.t, nav_e.h, '-b')
xlabel('Time [s]')
ylabel('[m]')
legend('REF', 'GNSS', 'INS/GNSS');
title('ALTITUDE');
grid

% POSITION ERRORS
[RN,RE]  = radius(nav_ref.lat, 'double');
LAT2M_1 = RN + nav_ref.h;
LON2M_1 = (RE + nav_ref.h).*cos(nav_ref.lat);

[RN,RE]  = radius(gnss.lat, 'double');
LAT2M_G = RN + gnss.h;
LON2M_G = (RE + gnss.h).*cos(gnss.lat);

[RN,RE]  = radius(gnss_ref.lat, 'double');
LAT2M_GR = RN + gnss_ref.h;
LON2M_GR = (RE + gnss_ref.h).*cos(gnss_ref.lat);

figure;
subplot(311)
plot(gnss_ref.t,  LAT2M_GR.*(gnss_ref.lat - ref_g.lat), '-c')
hold on
plot(nav_ref.t, LAT2M_1.*(nav_ref.lat - ref_n.lat), '-b')
plot (nav_e.tg, LAT2M_G.*sig3_rr(:,7), '--k', nav_e.tg, -LAT2M_G.*sig3_rr(:,7), '--k' )
xlabel('Time [s]')
ylabel('[m]')
legend('GNSS', 'INS/GNSS', '3\sigma');
title('LATITUDE ERROR');
grid

subplot(312)
plot(gnss_ref.t, LON2M_GR.*(gnss_ref.lon - ref_g.lon), '-c')
hold on
plot(nav_ref.t, LON2M_1.*(nav_ref.lon - ref_n.lon), '-b')
plot(nav_e.tg, LON2M_G.*sig3_rr(:,8), '--k', nav_e.tg, -LON2M_G.*sig3_rr(:,8), '--k' )
xlabel('Time [s]')
ylabel('[m]')
legend('GNSS', 'INS/GNSS', '3\sigma');
title('LONGITUDE ERROR');
grid

subplot(313)
plot(gnss_ref.t, (gnss_ref.h - ref_g.h), '-c')
hold on
plot(nav_ref.t, (nav_ref.h - ref_n.h), '-b')
plot(nav_e.tg, sig3_rr(:,9), '--k', nav_e.tg, -sig3_rr(:,9), '--k' )
xlabel('Time [s]')
ylabel('[m]')
legend('GNSS', 'INS/GNSS', '3\sigma');
title('ALTITUDE ERROR');
grid
