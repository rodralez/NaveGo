% Johann Diep (johann.diep@esa.int) - November 2021
%
% This script must be adapted each time an optimization needs to be run.
%
% It is noted that this containts rewriting the imu_structure.m script such
% that it accept parameter input. This functionality was removed after the
% optimal parameters were found.

clear; clc;

%% Optimization

Index_p1 = 1;
for p_1 = 1
    Index_p2 = 1;
    for p_2 = 1
        try
            imu_structure(p_1,p_2);

            load('gnss_planetary.mat'); load('imu_planetary.mat'); load('gnss_planetary_r.mat'); load('visual_planetary.mat');
            switch FusionCase
                case 'inertial_gnss'
                    nav_e = ins_gnss(imu_planetary,gnss_planetary,'dcm');
                case 'inertial_visual'
                    nav_e = ins_visual(imu_planetary,gnss_planetary_r,visual_planetary,'dcm');
                case 'inertial_visual_gnss'
                    nav_e = ins_visual_gnss(imu_planetary,gnss_planetary,visual_planetary,'dcm');
            end
            [nav_i,gnss_planetary_r] = navego_interpolation (nav_e, gnss_planetary_r);

            %% Optimization

            % Position Errors
            [RN,RE]  = radius(nav_i.lat);
            LAT2M = RN + nav_i.h;
            LON2M = (RE + nav_i.h).*cos(nav_i.lat);

            ErrorValue(Index_p1,Index_p2) = sqrt(rms(LAT2M.*(nav_i.lat - gnss_planetary_r.lat))^2 + rms(LON2M.*(nav_i.lon - gnss_planetary_r.lon))^2);
            ParameterValue(Index_p1,Index_p2,1) = p_1;
            ParameterValue(Index_p1,Index_p2,2) = p_2;
        catch
            disp('An error occured in the estimation.');
        end
        Index_p2 = Index_p2 + 1;
    end
    Index_p1 = Index_p1 + 1;
end