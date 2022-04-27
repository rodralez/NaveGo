% Johann Diep (johann.diep@esa.int) - November 2021
%
% This function builds the structure for the Canadian IMU dataset.
% It is written in order to allow for the optimization for the IMU
% parameters.
% 
% Input: 
% - p_arw: value for the angle random walks
% - p_vrw: value for the velocity random walks
% - p_gb_sta: value for  gyroscopic static biases
% - p_ab_sta: value for acceleration static biases
% - p_gb_dyn: value for gyroscopic dynamic biases
% - p_ab_dyn: value for acceleration dynamic biases
% - p_a_std: value for acceleration standard deviations
% - p_g_std: value for gyroscopic standard deviations
% - p_ab_psd: value for acceleration dynamic biases PSD
% - p_gb_psd: value for gyroscopic dynamic biases PSD
% - m_psd: value for magnetometer biases PSD
% 
% Output:
% - imu_planetary: struct containing the IMU data
%
% It is noted that in order to save time, this function was rewritten in
% its parameters for each optimization round (2 values were optimized each).
% This functionality was then removed for better overview.

function imu_planetary = imu_structure() 
    %% Conversion Constants

    G =  9.80665;       % Gravity constant, m/s^2
    G2MSS = G;          % g to m/s^2
    MSS2G = (1/G);      % m/s^2 to g
    D2R = (pi/180);     % degrees to radians
    R2D = (180/pi);     % radians to degrees
    KT2MS = 0.514444;   % knot to m/s
    MS2KMH = 3.6;       % m/s to km/h

    %% IMU ERROR PROFILE

    imu_data_planetary = readtable('../../external-data/Canada/IMU/imu.txt');

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
    %    ab_psd: 1x3 accrs dynamic biases PSD (m/s^2/root-Hz).
    %      freq: 1x1 sampling frequency (Hz).
    % ini_align: 1x3 initial attitude at t(1), [roll pitch yaw] (rad).
    % ini_align_err: 1x3 initial attitude errors at t(1), [roll pitch yaw] (rad).
    
    % Parameters with optimized values.
    p_arw = 8.0000e-05;
    p_vrw = 2.0000e-05;
    gb_sta = -0.0050;
    ab_sta = -0.0190;
    gb_dyn = 4.0000e-05;
    ab_dyn = 5.0000e-05;
    a_std = 0.001;
    g_std = 0.001;
    ab_psd = 0;
    gb_psd = 0;
    m_psd = 0;
    
    imu_planetary.arw      = [p_arw,p_arw,p_arw];             
    imu_planetary.arrw     = [0,0,0];                                                            
    imu_planetary.vrw      = [p_vrw,p_vrw,p_vrw];                        
    imu_planetary.vrrw     = [0,0,0];                                                             
    imu_planetary.gb_sta   = [gb_sta gb_sta gb_sta];             
    imu_planetary.ab_sta   = [ab_sta ab_sta ab_sta];               
    imu_planetary.gb_dyn   = [gb_dyn,gb_dyn,gb_dyn];    
    imu_planetary.ab_dyn   = [ab_dyn,ab_dyn,ab_dyn];  
    imu_planetary.gb_corr  = [1000 1000 1000];                                                     
    imu_planetary.ab_corr  = [1000 1000 1000];                                                       
    imu_planetary.a_std    = [a_std,a_std,a_std];
    imu_planetary.g_std    = [g_std,g_std,g_std];
    imu_planetary.ab_psd   = [ab_psd,ab_psd,ab_psd];
    imu_planetary.gb_psd   = [gb_psd,gb_psd,gb_psd];
    imu_planetary.m_psd    = [m_psd,m_psd,m_psd];

    % time 
%     timestamps = table2array(imu_data_planetary(:,1));
%     timestamps = (timestamps - timestamps(1));
%     dt = mean(diff(timestamps));
%     imu_planetary.t = (timestamps(1):dt:timestamps(end))';
%     imu_planetary.freq = 1/dt; 

    imu_planetary.ini_align_err = [10 10 10] .* D2R; % initial attitude align errors for matrix P in Kalman filter, [roll pitch yaw] (radians)
    imu_planetary.ini_align = [0 0 0] .* D2R; % Initial attitude align at t(1) (radians)

%     imu_planetary.fb = table2array(imu_data_planetary(:,2:4)); % acceleration measurements
%     imu_planetary.wb = table2array(imu_data_planetary(:,5:7)); % gyroscopic measurements

    %% Saving

    save('../data/imu_planetary.mat','imu_planetary');
end
