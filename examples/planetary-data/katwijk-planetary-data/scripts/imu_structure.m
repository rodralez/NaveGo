% Johann Diep (johann.diep@esa.int) - August 2021
%
% This script builds the structure for the Katwijk IMU dataset.

%% Conversion Constants

G =  9.80665;       % Gravity constant, m/s^2
G2MSS = G;          % g to m/s^2
MSS2G = (1/G);      % m/s^2 to g
D2R = (pi/180);     % degrees to radians
R2D = (180/pi);     % radians to degrees
KT2MS = 0.514444;   % knot to m/s
MS2KMH = 3.6;       % m/s to km/h

%% IMU ERROR PROFILE

imu_data_planetary = readtable('../../external-data/Katwijk/IMU/imu0.csv'); % read planetary IMU data

% IMU data structure:
%         t: Ix1 time vector (seconds).
%        fb: Ix3 accelerations vector in body frame XYZ (m/s^2).
%        wb: Ix3 turn rates vector in body frame XYZ (radians/s).
%       arw: 1x3 angle random walks (rad/s/root-Hz).
%      arrw: 1x3 angle rate random walks (rad/s^2/root-Hz).
%       vrw: 1x3 velocity random walks (m/s^2/root-Hz).
%      vrrw: 1x3 velocity rate random walks (m/s^3/root-Hz).
%     g_std: 1x3 gyros standard deviations (radians/s).
%     a_std: 1x3 accrs standard deviations (m/s^2).
%    gb_sta: 1x3 gyros static biases or turn-on biases (radians/s).
%    ab_sta: 1x3 accrs static biases or turn-on biases (m/s^2).
%    gb_dyn: 1x3 gyros dynamic biases or bias instabilities (radians/s).
%    ab_dyn: 1x3 accrs dynamic biases or bias instabilities (m/s^2).
%   gb_corr: 1x3 gyros correlation times (seconds).
%   ab_corr: 1x3 accrs correlation times (seconds).
%    gb_psd: 1x3 gyros dynamic biases PSD (rad/s/root-Hz).
%    ab_psd: 1x3 accrs dynamic biases PSD (m/s^2/root-Hz);
%      freq: 1x1 sampling frequency (Hz).
% ini_align: 1x3 initial attitude at t(1), [roll pitch yaw] (rad).
% ini_align_err: 1x3 initial attitude errors at t(1), [roll pitch yaw] (rad).

imu_planetary.arw      = [0.008017806365449,0.008107120830822,0.008142094400848];             
imu_planetary.arrw     = [0,0,0];                                                            
imu_planetary.vrw      = [0.001968757359111,0.002080847480414,0.001698689979355];              
imu_planetary.vrrw     = [0,0,0];                                                            
imu_planetary.gb_sta   = [0.007440407532675,0.044356514264794,0.062953158661843];             
imu_planetary.ab_sta   = [0.012290855897547,0.012470873935767,9.830174007054483];               
imu_planetary.gb_dyn   = [3.589857650346221e-04,3.897194991641202e-04,2.195082701413355e-04];    
imu_planetary.ab_dyn   = [2.936833659299578e-04,3.012048724015608e-04,2.990266040668724e-04];  
imu_planetary.gb_corr  = [1000 1000 1000];                                                     
imu_planetary.ab_corr  = [1000 1000 700];                                                       
imu_planetary.a_std    = [0.021363952880684,0.022855668192465,0.017875212169832];
imu_planetary.g_std    = [0.087135482784513,0.089545793244810,0.089085782298092];
imu_planetary.ab_psd   = [0.009287083472434,0.009524934391293,0.007911500297531];
imu_planetary.gb_psd   = [0.011352126650874,0.012324012659387,0.006941460988902];
imu_planetary.m_psd    = [0 0 0];

% time 
timestamps = table2array(imu_data_planetary(:,1));
timestamps = (timestamps - timestamps(1)) / 1e9;
dt = mean(diff(timestamps));
imu_planetary.t = (timestamps(1):dt:timestamps(end))';
imu_planetary.freq = 1/dt; 

imu_planetary.ini_align_err = [5 5 5] .* D2R; % initial attitude align errors for matrix P in Kalman filter, [roll pitch yaw] (radians)
imu_planetary.ini_align = [30 75 0] .* D2R; % Initial attitude align at t(1) (radians)

imu_planetary.fb = table2array(imu_data_planetary(:,5:7)); % acceleration measurements
imu_planetary.wb = table2array(imu_data_planetary(:,2:4)); % gyroscope measurements

%% Saving

save('../data/imu_planetary.mat','imu_planetary');