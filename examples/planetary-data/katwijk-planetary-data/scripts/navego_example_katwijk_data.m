% Johann Diep (johann.diep@esa.int) - November 2021
%
% This script runs the fusion of all sensor parameters. 

clear; clc; close all;

FusionCase = "inertial_visual_gnss";
Sparse = "true";

%% Generating Data

if FusionCase == "inertial_gnss"
    imu_structure;
    if Sparse == "true"
        gnss_sparse_structure;
    else
        gnss_structure;
    end
else
    imu_structure;
    visual_structure;
    if Sparse == "true"
        gnss_sparse_structure;
    else
        gnss_structure;
    end
end

%% Estimation

switch FusionCase
    case "inertial_gnss"
        nav_e = ins_gnss(imu_planetary,gnss_planetary,'dcm');
    case "inertial_visual"
        nav_e = ins_visual(imu_planetary,gnss_planetary_r,visual_planetary,'dcm');
    case "inertial_visual_gnss"
        nav_e = ins_visual_gnss(imu_planetary,gnss_planetary,visual_planetary,'dcm');
end

[nav_i,gnss_planetary_r] = navego_interpolation (nav_e, gnss_planetary_r);

if Sparse == "true"
    [gnss_i,gnss_planetary_r_sparse] = navego_interpolation(gnss_planetary,gnss_planetary_r_sparse);
else
    [gnss_i,gnss_planetary_r] = navego_interpolation(gnss_planetary,gnss_planetary_r);
end

%% Plotting

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
        plot(gnss_i.t,  LAT2M_GR.*(gnss_i.lat - gnss_planetary_r.lat), '.', 'Color', ones(1,3) * 0.75, 'LineWidth', 1.5)
        plot(nav_i.t, LAT2M.*(nav_i.lat - gnss_planetary_r.lat),'Color', [0, 0, 0], 'LineWidth', 1.5)
        grid on;
        xlabel('Time [s]')
        ylabel('[m]')
        legend('GNSS', 'IMU + degraded GNSS', 'Location', 'northoutside');
        title('Latitude Error');
        xlim([0,max(gnss_planetary.t)]);

        subplot(2,1,2);
        hold on;
        plot(gnss_i.t, LON2M_GR.*(gnss_i.lon - gnss_planetary_r.lon), '.', 'Color', ones(1,3) * 0.75, 'LineWidth', 1.5)
        plot(nav_i.t, LON2M.*(nav_i.lon - gnss_planetary_r.lon),'Color', [0, 0, 0], 'LineWidth', 1.5)
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
        plot(gnss_i.t,  LAT2M_GR.*(gnss_i.lat - gnss_planetary_r.lat), '.', 'Color', ones(1,3) * 0.75, 'LineWidth', 1.5)
        plot(nav_i.t, LAT2M.*(nav_i.lat - gnss_planetary_r.lat),'Color', [0, 0, 0], 'LineWidth', 1.5)
        grid on;
        xlabel('Time [s]')
        ylabel('[m]')
        legend('GNSS', 'IMU + OpenVINS', 'Location', 'northoutside');
        title('Latitude Error');
        xlim([0,max(gnss_planetary.t)]);

        subplot(2,1,2);
        hold on;
        plot(gnss_i.t, LON2M_GR.*(gnss_i.lon - gnss_planetary_r.lon), '.', 'Color', ones(1,3) * 0.75, 'LineWidth', 1.5)
        plot(nav_i.t, LON2M.*(nav_i.lon - gnss_planetary_r.lon),'Color', [0, 0, 0], 'LineWidth', 1.5)
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
        plot(gnss_i.t,  LAT2M_GR.*(gnss_i.lat - gnss_planetary_r.lat), '.', 'Color', ones(1,3) * 0.75, 'LineWidth', 1.5)
        plot(nav_i.t, LAT2M.*(nav_i.lat - gnss_planetary_r.lat),'Color', [0, 0, 0], 'LineWidth', 1.5)
        grid on;
        xlabel('Time [s]')
        ylabel('[m]')
        legend('GNSS', 'IMU + degraded GNSS + OpenVINS', 'Location', 'northoutside');
        title('Latitude Error');
        xlim([0,max(gnss_planetary.t)]);

        subplot(2,1,2);
        hold on;
        plot(gnss_i.t, LON2M_GR.*(gnss_i.lon - gnss_planetary_r.lon), '.', 'Color', ones(1,3) * 0.75, 'LineWidth', 1.5)
        plot(nav_i.t, LON2M.*(nav_i.lon - gnss_planetary_r.lon),'Color', [0, 0, 0], 'LineWidth', 1.5)
        grid on;
        xlabel('Time [s]')
        ylabel('[m]')
        legend('GNSS', 'IMU + degraded GNSS + OpenVINS', 'Location', 'northoutside');
        title('Longitude Error');
        xlim([0,max(gnss_planetary.t)]);
end