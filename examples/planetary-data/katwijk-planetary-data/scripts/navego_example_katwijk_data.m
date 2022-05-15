% navego_example_katwijk_data: post-processing integration of
% IMU, GNSS and visual data for the Katwijk dataset.
%
% The main goal is to integrate IMU and GNSS measurements from Ekinox-D
% sensor which includes both IMU and GNSS sensors.
%
% Sensors dataset was generated driving a car through the streets of
% Turin city (Italy).
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
% References:
%
%
% Version: 001
% Date:    2021/11/01
% Author:  Johann Diep <johanndiep@gmail.com>
% URL:     https://github.com/rodralez/navego
%
% NOTE: NaveGo assumes that IMU is aligned with respect to body-frame as
% X-forward, Y-right and Z-down.
%
% NOTE: NaveGo assumes that yaw angle (heading) is positive clockwise.

clc
close all
clear
matlabrc

addpath ../../../../ins/
addpath ../../../../ins-gnss/
addpath ../../../../ins-visual/
addpath ../../../../ins-visual-gnss/
addpath ../../../../conversions/
addpath ../../../../performance-analysis/
addpath ../../../../plot/
addpath ../../../../misc/
addpath ../../../../simulation/
addpath ../data/

navego_print_version;

fprintf('\nNaveGo: starting Katwijk planetary data integration... \n')

%% PARAMETERS

% Comment any of the following parameters in order to NOT execute a
% particular portion of code

GEN_DATA     = 'ON';

if (~exist('GEN_DATA','var')), GEN_DATA = 'OFF'; end

% FusionCase = "inertial_gnss";
% FusionCase = "inertial_visual";
FusionCase = "inertial_visual_gnss";

Sparse = "true";
% Sparse = "false";

fprintf('NaveGo: parameter FusionCase = %s \n', FusionCase)
fprintf('NaveGo: parameter Sparse = %s \n', Sparse)

%% Generating Data

%% Generating Data

if strcmp(GEN_DATA, 'ON')

    if FusionCase == "inertial_gnss"
        fprintf('NaveGo: generating IMU data... \n')
        imu_structure;

        fprintf('NaveGo: generating GNSS data... \n')
        if Sparse == "true"
            gnss_sparse_structure;
        else
            gnss_structure;
        end
    else
        imu_structure;
        fprintf('NaveGo: generating IMU data... \n')
        visual_structure;
        fprintf('NaveGo: generating VISUAL data... \n')
        if Sparse == "true"
            fprintf('NaveGo: generating GNSS data... \n')
            gnss_sparse_structure;
        else
            gnss_structure;
        end
    end
else
    fprintf('NaveGo: loading data... \n')

    if FusionCase == "inertial_gnss"
        %         imu_structure;
        load imu_planetary
        if Sparse == "true"
            %             gnss_sparse_structure;
            load gnss_planetary_r
            load gnss_planetary_sparse_r
            load gnss_planetary
        else
            %             gnss_structure;
            load gnss_planetary_r
            load gnss_planetary
        end
    else
        %         imu_structure;
        load imu_planetary
        %         visual_structure;
        load visual_planetary
        if Sparse == "true"
            %             gnss_sparse_structure;
            load gnss_planetary_r
            load gnss_planetary_sparse_r
            load gnss_planetary
        else
            %             gnss_structure;
            load gnss_planetary_r
            load gnss_planetary
        end
    end
end

%% Estimation

switch FusionCase
    case "inertial_gnss"
        fprintf('NaveGo: processing INS/GNSS integration... \n')
        nav_e = ins_gnss(imu_planetary,gnss_planetary,'dcm');
    case "inertial_visual"
        fprintf('NaveGo: processing INS/VISUAL integration... \n')
        nav_e = ins_visual(imu_planetary,gnss_planetary_r,visual_planetary,'dcm'); % note: figure out why we input gnss_planetary_r
    case "inertial_visual_gnss"
        fprintf('NaveGo: processing INS/GNSS/VISUAL integration... \n')
        nav_e = ins_visual_gnss(imu_planetary,gnss_planetary,visual_planetary,'dcm');
end

[nav_i, nav_ref] = navego_interpolation (nav_e, gnss_planetary_r);

if Sparse == "true"
    [gnss_i,gnss_ref] = navego_interpolation(gnss_planetary,gnss_planetary_sparse_r);
else
    [gnss_i,gnss_ref] = navego_interpolation(gnss_planetary,gnss_planetary_r);
end

%% Plotting

fprintf('NaveGo: plotting... \n')

switch FusionCase
    %% Plotting: IMU + GNSS
    case "inertial_gnss"

        % Position
        figure();
        hold on;
        plot(rad2deg(gnss_planetary.lon),rad2deg(gnss_planetary.lat),'.', 'Color', ones(1,3) * 0.75, 'LineWidth', 1.5);
        scatter(rad2deg(gnss_planetary_r.lon),rad2deg(gnss_planetary_r.lat),'y.','MarkerEdgeAlpha',.6);
        plot(rad2deg(nav_e.lon),rad2deg(nav_e.lat),'Color', [0, 0, 0], 'LineWidth', 1.5);
        grid on;
        xlabel('Longitude [deg]');
        ylabel('Latitude [deg]');
        legend('degraded GNSS','RTK','IMU + degraded GNSS','Location','Southeast');
        axis equal;

        % Position Errors
        [RN,RE]  = radius(nav_i.lat);
        LAT2M = RN + nav_i.h;
        LON2M = (RE + nav_i.h).*cos(nav_i.lat);
        [RN,RE]  = radius(gnss_i.lat);
        LAT2M_GR = RN + gnss_i.h;
        LON2M_GR = (RE + gnss_i.h).*cos(gnss_i.lat);

        figure();
        subplot(2,1,1);
        hold on;
        plot(gnss_i.t,LAT2M_GR.*(gnss_i.lat - gnss_ref.lat), '.', 'Color', ones(1,3) * 0.75, 'LineWidth', 1.5)
        plot(nav_i.t, LAT2M.*(nav_i.lat - nav_ref.lat),'Color', [0, 0, 0], 'LineWidth', 1.5)
        grid on;
        xlabel('Time [s]')
        ylabel('[m]')
        legend('GNSS', 'IMU + degraded GNSS', 'Location', 'northoutside');
        title('Latitude Error');
        xlim([0,max(gnss_planetary.t)]);
        subplot(2,1,2);
        hold on;
        plot(gnss_i.t, LON2M_GR.*(gnss_i.lon - gnss_ref.lon), '.', 'Color', ones(1,3) * 0.75, 'LineWidth', 1.5)
        plot(nav_i.t, LON2M.*(nav_i.lon - nav_ref.lon),'Color', [0, 0, 0], 'LineWidth', 1.5)
        grid on;
        xlabel('Time [s]')
        ylabel('[m]')
        legend('GNSS', 'IMU + degraded GNSS', 'Location', 'northoutside');
        title('Longitude Error');
        xlim([0,max(gnss_planetary.t)]);

        %% Plotting: Vision + IMU
    case "inertial_visual"

        % Position
        figure();
        hold on;
        plot(rad2deg(visual_planetary.lon),rad2deg(visual_planetary.lat),'.', 'Color', ones(1,3) * 0.75, 'LineWidth', 1.5);
        scatter(rad2deg(gnss_planetary_r.lon),rad2deg(gnss_planetary_r.lat),'y.','MarkerEdgeAlpha',.6);
        plot(rad2deg(nav_e.lon),rad2deg(nav_e.lat),'Color', [0, 0, 0], 'LineWidth', 1.5);
        grid on;
        xlabel('Longitude');
        ylabel('Latitude');
        legend('OpenVINS','RTK','IMU + OpenVINS','Location','Southeast');
        axis equal;
        % Position Errors
        [RN,RE]  = radius(nav_i.lat);
        LAT2M = RN + nav_i.h;
        LON2M = (RE + nav_i.h).*cos(nav_i.lat);

        [RN,RE]  = radius(gnss_i.lat);
        LAT2M_GR = RN + gnss_i.h;
        LON2M_GR = (RE + gnss_i.h).*cos(gnss_i.lat);

        figure();
        subplot(2,1,1);
        hold on;
        plot(gnss_i.t,  LAT2M_GR.*(gnss_i.lat - gnss_ref.lat), '.', 'Color', ones(1,3) * 0.75, 'LineWidth', 1.5)
        plot(nav_i.t, LAT2M.*(nav_i.lat - nav_ref.lat),'Color', [0, 0, 0], 'LineWidth', 1.5)
        grid on;
        xlabel('Time [s]')
        ylabel('[m]')
        legend('GNSS', 'IMU + OpenVINS', 'Location', 'northoutside');
        title('Latitude Error');
        xlim([0,max(gnss_planetary.t)]);
        subplot(2,1,2);
        hold on;
        plot(gnss_i.t, LON2M_GR.*(gnss_i.lon - gnss_i.lon), '.', 'Color', ones(1,3) * 0.75, 'LineWidth', 1.5)
        plot(nav_i.t, LON2M.*(nav_i.lon - nav_ref.lon),'Color', [0, 0, 0], 'LineWidth', 1.5)
        grid on;
        xlabel('Time [s]')
        ylabel('[m]')
        legend('GNSS', 'IMU + OpenVINS', 'Location', 'northoutside');
        title('Longitude Error');
        xlim([0,max(gnss_planetary.t)]);

        %% Plotting: Vision + GNSS + INS
    case "inertial_visual_gnss"

        % Position
        figure();
        hold on;
        plot(rad2deg(gnss_planetary.lon),rad2deg(gnss_planetary.lat),'.', 'Color', ones(1,3) * 0.75, 'LineWidth', 1.5);
        scatter(rad2deg(gnss_planetary_r.lon),rad2deg(gnss_planetary_r.lat),'y.','MarkerEdgeAlpha',.6);
        plot(rad2deg(nav_e.lon),rad2deg(nav_e.lat),'Color', [0, 0, 0], 'LineWidth', 1.5);
        grid on;
        xlabel('Longitude [deg]');
        ylabel('Latitude [deg]');
        legend('degraded GNSS','RTK','IMU + degraded GNSS + OpenVINS','Location','Southeast');
        axis equal;

        % Position Errors
        [RN,RE]  = radius(nav_i.lat);
        LAT2M = RN + nav_i.h;
        LON2M = (RE + nav_i.h).*cos(nav_i.lat);

        [RN,RE]  = radius(gnss_i.lat);
        LAT2M_GR = RN + gnss_i.h;
        LON2M_GR = (RE + gnss_i.h).*cos(gnss_i.lat);

        figure();
        subplot(2,1,1);
        hold on;
        plot(gnss_i.t,  LAT2M_GR.*(gnss_i.lat - gnss_ref.lat), '.', 'Color', ones(1,3) * 0.75, 'LineWidth', 1.5)
        plot(nav_i.t, LAT2M.*(nav_i.lat - nav_ref.lat),'Color', [0, 0, 0], 'LineWidth', 1.5)
        grid on;
        xlabel('Time [s]')
        ylabel('[m]')
        legend('GNSS', 'IMU + degraded GNSS + OpenVINS', 'Location', 'northoutside');
        title('Latitude Error');
        xlim([0,max(gnss_planetary.t)]);
        subplot(2,1,2);
        hold on;
        plot(gnss_i.t, LON2M_GR.*(gnss_i.lon - gnss_ref.lon), '.', 'Color', ones(1,3) * 0.75, 'LineWidth', 1.5)
        plot(nav_i.t, LON2M.*(nav_i.lon - nav_ref.lon),'Color', [0, 0, 0], 'LineWidth', 1.5)
        grid on;
        xlabel('Time [s]')
        ylabel('[m]')
        legend('GNSS', 'IMU + degraded GNSS + OpenVINS', 'Location', 'northoutside');
        title('Longitude Error');
        xlim([0,max(gnss_planetary.t)]);
end