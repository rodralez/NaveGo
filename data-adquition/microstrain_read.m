function ustrain = microstrain_read(file)
% microstrain_read: reads .cvs files created by Microstrain software. 
%
% INPUT:
%   fname, file name (string).
%
% OUTPUT
%   ustrain, fields in data structure depends on user selection. In this 
%   function several fields have been added but may be modified to 
%   comply to present fields in .cvs file. Please, read the 
%   3DM-GX3 manual for more information about existing fields.
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
%   Reference:
%       Microstrain. 3DM-GX3®-35 Data Communications Protocol. 8500-0014 
%       Revision 010. Oct. 1 2013.
%
% Version: 001
% Date:    2018/09/12
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

%% CONSTANTS

G =   9.80665;
D2R = pi/180;

%% OPEN FILE

ustrain_f = fopen(file, 'r');
if ustrain_f == -1
    error('microstrain_read: ERROR: %s not found', file)
end

%% TOTAL NUMBER OF LINES

lines = nnz(fread(ustrain_f) == 10);
fprintf('microstrain_read: %s file has %d lines. \n', file, lines);

% Set pointer back to the beginning
fseek(ustrain_f,0,'bof');

%% DATA STRUCTURE

%  4 GPS TFlags, GPS Week, GPS TOW, IMU Timestamp [x800E], 
%  3 IMU Sync Flags [x800F], IMU Sync Seconds [x800F], IMU Sync Nanoseconds [x800F], 
%  3 X Accel [x8004],Y Accel [x8004],Z Accel [x8004], 
%  3 X Gyro [x8005],Y Gyro [x8005],Z Gyro [x8005],
%  3 X Mag [x8006],Y Mag [x8006],Z Mag [x8006], 
%  3 Delta Theta X [x8007], Delta Theta Y [x8007], Delta Theta Z [x8007], 
%  3 Delta Vel X [x8008],Delta Vel Y [x8008],Delta Vel Z [x8008],
%  9 M11 [x8009],M12 [x8009],M13 [x8009],M21 [x8009],M22 [x8009],M23 [x8009],M31 [x8009],M32 [x8009],M33 [x8009],
%  4 q0 [x800A],q1 [x800A],q2 [x800A],q3 [x800A],
%  9 C11 [x800B],C12 [x800B],C13 [x800B],C21 [x800B],C22 [x800B],C23 [x800B],C31 [x800B],C32 [x800B],C33 [x800B],
%  3 Roll [x800C],Pitch [x800C],Yaw [x800C],
%  3 North X [x8010], North Y [x8010], North Z [x8010], 
%  3 Up X [x8011],Up Y [x8011],Up Z [x8011],
%  4 Lat [x8103],Long [x8103],Height [x8103],MSL Height [x8103],
%  3 Horz Acc [x8103],Vert Acc [x8103],Flags [x8103], 
%  5 ECEF X [x8104],ECEF Y [x8104],ECEF Z [x8104], ECEF Acc [x8104], Flags [x8104], 
%  9 Vel N [x8105], Vel E [x8105], Vel D [x8105], Speed [x8105], Gnd Speed [x8105], Heading [x8105],Speed Acc [x8105],Heading Acc [x8105],Flags [x8105],
%  5 ECEF Vel X [x8106],ECEF Vel Y [x8106],ECEF Vel Z [x8106],ECEF Vel Acc [x8106],Flags [x8106],
%  8 Geo DOP [x8107],Pos DOP [x8107],Hor DOP [x8107],Vert DOP [x8107],Time DOP [x8107],Northing DOP [x8107],Easting DOP [x8107],Flags [x8107],
%  8 UTC Year [x8108],UTC Month [x8108],UTC Day [x8108],UTC Hour [x8108],UTC Minute [x8108],UTC Second [x8108],UTC Millesecond [x8108],Flags [x8108],
%  4 Clock Bias [x810A],Clock Drift [x810A],Clock Acc,Flags [x810A],
%  4 GPS Fix [x810B],GPS SVs Used [x810B],GPS Fix Flags [x810B],Flags [x810B],
%  7 SVI Channel [x810C],SVI ID [x810C],SVI CNR [x810C],SVI Azimuth [x810C],SVI Elev [x810C],SVI Flags [x810C],[x810C],
%  4 HW Sensor Stat [x810D],HW Ant Stat [x810D],HW Ant Pwr [x810D],Flags [x810D]

ustrain = struct;
ustrain_fields = {'gps_flags', 'week', 't', 'imu_timestamp', 'imu_sync_flags', 'imu_sync_s', 'imu_sync_ns', ... 
    'fb', 'wb', 'mb', 'delta_theta', 'delta_vel', ...
    'M', 'q', 'C', 'roll', 'pitch', 'yaw', ... 
    'mb_stab', 'fb_stab', 'lat', 'lon', 'h', 'h_msl', 'hor_accu', 'vert_accu', 'pos_vld_flags', 'ecef', 'ecef_accu', 'ecef_vld_flags', ... 
    'vel_ned', 'speed_vald', 'gnd_speed', 'heading', 'speed_accu', 'heading_accu', 'vel_ned_vld_flags', 'vel_ecef', 'vel_ecef_accu', 'vel_ecef_vld_flags', ... 
    'dop_geo', 'dop_pos', 'dop_hor', 'dop_vert', 'dop_time', 'dop_north', 'dop_east', 'dop_vld_flags', ... 
    'utc', 'gps_clk_bias', 'gps_clk_drift', 'gps_clk_accu', 'gps_clk_vld_flags', ... 
    'gps_fix_type', 'gps_sv', 'gps_fix_flags', 'svi_channel', 'svi_id', 'svi_cnr', 'svi_azi', ... 
    'svi_elev', 'svi_flags', 'hw_sensor_stat', 'hw_ant_stat', 'hw_ant_pwr'};

for i=1:55
    ustrain.(ustrain_fields{i}) = 0;
end

%% Total number of columns

% Copy and paste first data row
commas = strfind('7,1970,490025.066688,115478418,7,1848,66688000,0.05480965,0.03887069,-0.99567312,-0.0003955,-0.00398797,0.00066199,-0.14926282,-0.18364368,-0.07096149,-0.00002181,-0.00000011,-0.00002604,0.0004549,0.00042884,-0.00986408,-0.62398618,0.78054726,-0.03724382,-0.77954715,-0.62508231,-0.03972664,-0.05428897,0.00424444,0.9985162,0.43285319,-0.02539607,-0.00984465,0.90105283,1,-0.00002604,0.00000011,0.00002604,1,-0.00002181,-0.00000011,0.00002181,1,-0.0397647,0.03725244,2.24518442,-0.14811563,-0.1854372,-0.0712141,0.03723363,0.03971577,-0.99824303,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,', ',');
col = max(size(commas)) + 1;

% Create pattern for textscan()
str = [];
for i=1:col
    
    str = [str '%f '] ;
end

%% HEADER

rows = 16;

ustrain_h = cell(rows,1);
for i =1:rows
    ustrain_h{i,1} = fgets(ustrain_f);
end

%% DATA

data_cell = textscan(ustrain_f, str, 'Delimiter',',','EmptyValue',0);
data_m1 = cell2mat (data_cell);

%% IMU data

% Delete rows where GPS TOW equals zero
idx = data_m1(:,3) ~= 0;
data_m2 = data_m1(idx , :);

idx = data_m2(:,4) ~= 0;
data_vld = data_m2(idx , :);

ustrain.gps_flags           = data_vld(:, 1);
ustrain.week                = data_vld(:, 2);
ustrain.t                   = data_vld(:, 3);
ustrain.imu_timestamp       = data_vld(:, 4);

ustrain.imu_sync_flags      = data_vld(:, 5);
ustrain.imu_sync_s          = data_vld(:, 6);
ustrain.imu_sync_ns         = data_vld(:, 7);

ustrain.fb                  = data_vld(:, 8:10) * G;
ustrain.wb                  = data_vld(:, 11:13);
ustrain.mb                  = data_vld(:, 14:16);
ustrain.delta_theta         = data_vld(:, 17:19);
ustrain.delta_vel           = data_vld(:, 20:22);

ustrain.M                   = data_vld(:, 23:31);
ustrain.q                   = data_vld(:, 32:35);
ustrain.C                   = data_vld(:, 36:44);

ustrain.roll                = data_vld(:, 45);
ustrain.pitch               = data_vld(:, 46);
ustrain.yaw                 = data_vld(:, 47);
ustrain.mb_stab             = data_vld(:, 48:50);
ustrain.fb_stab             = data_vld(:, 51:53);

ustrain.lat                 = data_vld(:, 54);
ustrain.lon                 = data_vld(:, 55);
ustrain.h                   = data_vld(:, 56);
ustrain.h_msl               = data_vld(:, 57);
ustrain.hor_accu            = data_vld(:, 58);
ustrain.vert_accu           = data_vld(:, 59);
ustrain.pos_vld_flags       = data_vld(:, 60); 

ustrain.ecef                = data_vld(:, 61:63);
ustrain.ecef_accu           = data_vld(:, 64);
ustrain.ecef_vld_flags      = data_vld(:, 65); 

ustrain.vel_ned             = data_vld(:, 66:68);
ustrain.speed_vald          = data_vld(:, 69); 
ustrain.gnd_speed           = data_vld(:, 70); 
ustrain.heading             = data_vld(:, 71); 
ustrain.speed_accu          = data_vld(:, 72); 
ustrain.heading_accu        = data_vld(:, 73); 
ustrain.vel_ned_vld_flags   = data_vld(:, 74); 

ustrain.vel_ecef            = data_vld(:, 75:77); 
ustrain.vel_ecef_accu       = data_vld(:, 78); 
ustrain.vel_ecef_vld_flags  = data_vld(:, 79); 

ustrain.dop_geo             = data_vld(:, 80); 
ustrain.dop_pos             = data_vld(:, 81); 
ustrain.dop_hor             = data_vld(:, 82); 
ustrain.dop_vert            = data_vld(:, 83);
ustrain.dop_time            = data_vld(:, 84);
ustrain.dop_north           = data_vld(:, 85);
ustrain.dop_east            = data_vld(:, 86);
ustrain.dop_vld_flags       = data_vld(:, 87);

ustrain.utc                 = data_vld(:, 88:94);
ustrain.utc_vld_flags       = data_vld(:, 95);

ustrain.gps_clk_bias        = data_vld(:, 96);
ustrain.gps_clk_drift       = data_vld(:, 97);
ustrain.gps_clk_accu        = data_vld(:, 98);
ustrain.gps_clk_vld_flags   = data_vld(:, 99);

ustrain.gps_fix_type        = data_vld(:, 100);
ustrain.gps_sv              = data_vld(:, 101);
ustrain.gps_fix_flags       = data_vld(:, 102);
ustrain.gps_fix_vld_flags   = data_vld(:, 103);

ustrain.svi_channel         = data_vld(:, 104);
ustrain.svi_id              = data_vld(:, 105);
ustrain.svi_cnr             = data_vld(:, 106);
ustrain.svi_azi             = data_vld(:, 107);
ustrain.svi_elev            = data_vld(:, 108);
ustrain.svi_flags           = data_vld(:, 109);
% ustrain.svi_dumb            = data_imu(:, 110);

ustrain.hw_sensor_stat      = data_vld(:, 111);
ustrain.hw_ant_stat         = data_vld(:, 112);
ustrain.hw_ant_pwr          = data_vld(:, 113);
ustrain.hw_ant_vld_flags    = data_vld(:, 114);

% Frequency
dt = median(diff(ustrain.t));
ustrain.freq = round(1/dt);

% Header
ustrain.header = ustrain_h;

% NED coordinates, check if gravity is negative. 
ustrain = correct_gravity(ustrain);

fclose(ustrain_f);

end

