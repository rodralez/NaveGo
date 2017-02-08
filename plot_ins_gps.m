function plot_ins_gps(ins_gps, gps, ref )
% plot_ins_gps: plots several figures relate to INS/GPS performance.
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
% Date:    2016/11/08
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

global R2D

sig3_rr = abs(ins_gps.P_d.^(0.5)).*3;

%% TRAJECTORY

figure

plot3(ref.lon.*R2D, ref.lat.*R2D, ref.h)
hold on
plot3(ref.lon(1).*R2D, ref.lat(1).*R2D, ref.h(1), 'or', 'MarkerSize', 10, 'LineWidth', 2)
axis tight
title('TRAJECTORY')
xlabel('Longitude [deg.]')
ylabel('Latitude [deg.]')
zlabel('Altitude [m]')
grid

%% ATTITUDE

figure

subplot(311)
plot(ref.t, R2D.*ref.roll, '--k', ins_gps.t, R2D.*ins_gps.roll,'-b'), hold on,
%     plot(ustrain_dyn.t, R2D.*ustrain_dyn.roll, '--r')
ylabel('[deg]')
xlabel('Time [s]')
legend('REF', 'INS/GPS');
title('ROLL');

subplot(312)
plot(ref.t, R2D.*ref.pitch, '--k', ins_gps.t, R2D.*ins_gps.pitch,'-b');
ylabel('[deg]')
xlabel('Time [s]')
legend('REF', 'INS/GPS');
title('PITCH');

subplot(313)
plot(ref.t, R2D.* ref.yaw, '--k', ins_gps.t, R2D.* ins_gps.yaw,'-b');
%     hold on,
%     plot(gps.t, R2D.* gps.cog,'-r');
ylabel('[deg]')
xlabel('Time [s]')
legend('REF', 'INS/GPS');
title('YAW');

%% ATTITUDE ERRORS

figure

subplot(311)
plot(ins_gps.t, (ins_gps.roll-ref.roll).*R2D, '-b');
hold on
plot (gps.t, R2D.*sig3_rr(:,1), '--k', gps.t, -R2D.*sig3_rr(:,1), '--k' )
ylabel('[deg]')
xlabel('Time [s]')
legend('INS/GPS', '3\sigma');
title('ROLL ERROR');

subplot(312)
plot(ins_gps.t, (ins_gps.pitch-ref.pitch).*R2D, '-b');
hold on
plot (gps.t, R2D.*sig3_rr(:,2), '--k', gps.t, -R2D.*sig3_rr(:,2), '--k' )
ylabel('[deg]')
xlabel('Time [s]')
legend('INS/GPS', '3\sigma');
title('PITCH ERROR');

subplot(313)
plot(ins_gps.t, (ins_gps.yaw-ref.yaw).*R2D, '-b');
hold on
plot (gps.t, R2D.*sig3_rr(:,3), '--k', gps.t, -R2D.*sig3_rr(:,3), '--k' )
ylabel('[deg]')
xlabel('Time [s]')
legend('INS/GPS', '3\sigma');
title('YAW ERROR');

%% VELOCITIES

if (isfield(ref, 'vel') & isfield(ref, 'vel'))
    
    figure
    
    subplot(311)
    plot(ref.t, ref.vel(:,1), '--k', gps.t, gps.vel(:,1),'-g', ins_gps.t, ins_gps.vel(:,1),'-b');
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('REF', 'GPS', 'INS/GPS');
    title('NORTH VELOCITY');
    
    subplot(312)
    plot(ref.t, ref.vel(:,2), '--k', gps.t, gps.vel(:,2),'-g', ins_gps.t, ins_gps.vel(:,2),'-b');
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('REF', 'GPS', 'INS/GPS');
    title('EAST VELOCITY');
    
    subplot(313)
    plot(ref.t, ref.vel(:,3), '--k', gps.t, gps.vel(:,3),'-g', ins_gps.t, ins_gps.vel(:,3),'-b');
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('REF', 'GPS', 'INS/GPS');
    title('DOWN VELOCITY');
    
    %% VELOCITIES ERRORS
    
    figure
    
    subplot(311)
    plot(gps.t, (gps.vel(:,1)-gps.vel(:,1)), '-g');
    hold on
    plot(ins_gps.t, (ins_gps.vel(:,1)-ref.vel(:,1)), '-b');
    hold on
    plot (gps.t, sig3_rr(:,4), '--k', gps.t, -sig3_rr(:,4), '--k' )
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('GPS', 'INS/GPS', '3\sigma');
    title('VELOCITY NORTH ERROR');
    
    subplot(312)
    plot(gps.t, (gps.vel(:,2)-gps.vel(:,2)), '-g');
    hold on
    plot(ins_gps.t, (ins_gps.vel(:,2)-ref.vel(:,2)), '-b');
    hold on
    plot (gps.t, sig3_rr(:,5), '--k', gps.t, -sig3_rr(:,5), '--k' )
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('GPS', 'INS/GPS', '3\sigma');
    title('VELOCITY EAST ERROR');
    
    subplot(313)
    plot(gps.t, (gps.vel(:,3)-gps.vel(:,3)), '-g');
    hold on
    plot(ins_gps.t, (ins_gps.vel(:,3)-ref.vel(:,3)), '-b');
    hold on
    plot (gps.t, sig3_rr(:,6), '--k', gps.t, -sig3_rr(:,6), '--k' )
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('GPS', 'INS/GPS', '3\sigma');
    title('VELOCITY DOWN ERROR');
    
end

%% POSITION

figure

subplot(311)
plot(ref.t, ref.lat .*R2D, '--k', gps.t, gps.lat.*R2D, '-g', ins_gps.t, ins_gps.lat.*R2D, '-b');
xlabel('Time [s]')
ylabel('[deg]')
legend('REF', 'GPS', 'INS/GPS');
title('LATITUDE');

subplot(312)
plot(ref.t, ref.lon .*R2D, '--k', gps.t, gps.lon.*R2D, '-g', ins_gps.t, ins_gps.lon.*R2D, '-b');
xlabel('Time [s]')
ylabel('[deg]')
legend('REF', 'GPS', 'INS/GPS');
title('LONGITUDE');

subplot(313)
plot(ref.t, ref.h, '--k', gps.t, gps.h, '-g', ins_gps.t, ins_gps.h, '-b');
xlabel('Time [s]')
ylabel('[m]')
legend('REF', 'GPS', 'INS/GPS');
title('ALTITUDE');

%% POSITION ERRORS

[RN,RE]  = radius(ins_gps.lat, 'double');
lat2m = RN + ins_gps.h;
lon2m = (RE + ins_gps.h).*cos(ins_gps.lat);

[RN,RE]  = radius(gps.lat, 'double');
lat2m_g = RN + gps.h;
lon2m_g = (RE + gps.h).*cos(gps.lat);

figure

subplot(311)
plot(gps.t, lat2m_g.*(gps.lat - gps.lat), '-g')
hold on
plot(ins_gps.t, lat2m.*(ins_gps.lat - ref.lat), '-b')
hold on
plot (gps.t, lat2m_g.*sig3_rr(:,7), '--k', gps.t, -lat2m_g.*sig3_rr(:,7), '--k' )
xlabel('Time [s]')
ylabel('[m]')
legend('GPS', 'INS/GPS', '3\sigma');
title('LATITUDE ERROR');

subplot(312)
plot(gps.t, lon2m_g.*(gps.lon - gps.lon), '-g')
hold on
plot(ins_gps.t, lon2m.*(ins_gps.lon - ref.lon), '-b')
hold on
plot(gps.t, lon2m_g.*sig3_rr(:,8), '--k', gps.t, -lon2m_g.*sig3_rr(:,8), '--k' )
xlabel('Time [s]')
ylabel('[m]')
legend('GPS', 'INS/GPS', '3\sigma');
title('LONGITUDE ERROR');

subplot(313)
plot(gps.t, (gps.h - gps.h), '-g')
hold on
plot(ins_gps.t, (ins_gps.h - ref.h), '-b')
hold on
plot(gps.t, sig3_rr(:,9), '--k', gps.t, -sig3_rr(:,9), '--k' )
xlabel('Time [s]')
ylabel('[m]')
legend('GPS', 'INS/GPS', '3\sigma');
title('ALTITUDE ERROR');

end