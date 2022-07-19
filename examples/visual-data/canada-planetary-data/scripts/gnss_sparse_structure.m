% Johann Diep (johann.diep@esa.int) - November 2021
%
% This script builds the structure for the Canadian GNSS dataset.

warning off;

%% Conversion Constants

G =  9.80665;       % Gravity constant, m/s^2
G2MSS = G;          % g to m/s^2
MSS2G = (1/G);      % m/s^2 to g
D2R = (pi/180);     % degrees to radians
R2D = (180/pi);     % radians to degrees
KT2MS = 0.514444;   % knot to m/s
MS2KMH = 3.6;       % m/s to km/h

%% GNSS ERROR PROFILE

gnss_data_planetary = readtable('./external-data/canada/gnss/global-pose-utm.txt');
load('./canada-planetary-data/data/imu_planetary.mat');

% GNSS data structure:
%         t: Mx1 time vector (seconds).
%       lat: Mx1 latitude (radians).
%       lon: Mx1 longitude (radians).
%         h: Mx1 altitude (m).
%       vel: Mx3 NED velocities (m/s).
%       std: 1x3 position standard deviations, [lat lon h] (rad, rad, m).
%      stdm: 1x3 position standard deviations, [lat lon h] (m, m, m).
%      stdv: 1x3 velocity standard deviations, [Vn Ve Vd] (m/s).
%      larm: 3x1 lever arm from IMU to GNSS antenna (x-fwd, y-right, z-down) (m).
%      freq: 1x1 sampling frequency (Hz).
%   zupt_th: 1x1 ZUPT threshold (m/s).
%  zupt_win: 1x1 ZUPT time window (seconds).
%       eps: 1x1 time interval to compare IMU time vector to GNSS time vector (seconds).

gnss_planetary.stdm = 1 * [1 1 1]; % degradation

% GNSS positions
[gnss_planetary.lat,gnss_planetary.lon] = utm2deg(table2array(gnss_data_planetary(:,2)),table2array(gnss_data_planetary(:,3)),repelem(['18 T';'18 T'],400,1));
gnss_planetary.lat = gnss_planetary.lat.*D2R;
gnss_planetary.lon = gnss_planetary.lon.*D2R;
gnss_planetary.h = table2array(gnss_data_planetary(:,4));

gnss_planetary = gnss_m2r(gnss_planetary.lat(1),gnss_planetary.h(1),gnss_planetary); % convert error from meters to radians

% parsing the timestamps of the measurements
timestamps = table2array(gnss_data_planetary(:,1));
gnss_planetary.t = (timestamps - timestamps(1));

gnss_planetary.freq = 1/mean(diff(gnss_planetary.t)); % estimating the frequency
[gnss_planetary.vel,~] = pllh2vned(gnss_planetary); % estimating the velocity

gnss_planetary.larm = [-0.156,0.511,0.004]'; % GNSS lever arm from IMU to GNSS antenna
gnss_planetary.eps = mean(diff(imu_planetary.t))/3; % rule of thumb for choosing eps
gnss_planetary.stdv = gnss_planetary.stdm/100; % educated guess

% Parameters for ZUPT detection algorithm  copied from above
gnss_planetary.zupt_th = 0; % ZUPT threshold (m/s).
gnss_planetary.zupt_win = 4; % ZUPT time window (seconds).

% Increase frequency by interpolation
t_q = 0:0.05:gnss_planetary.t(end);
lon_q = interp1(gnss_planetary.t,gnss_planetary.lon,t_q);
lat_q = interp1(gnss_planetary.t,gnss_planetary.lat,t_q);
h_q = interp1(gnss_planetary.t,gnss_planetary.h,t_q);
v_q = interp1(gnss_planetary.t,gnss_planetary.vel,t_q);
freq_q = 1/mean(diff(t_q));

gnss_planetary.t = t_q'; 
gnss_planetary.lon = lon_q';
gnss_planetary.lat = lat_q';
gnss_planetary.h = h_q';
gnss_planetary.vel = v_q;
gnss_planetary.freq = freq_q;

gnss_planetary_r = gnss_planetary; % reference

% remove datapoints
gnss_planetary.t(2000:4000) = []; 
gnss_planetary.lon(2000:4000) = [];
gnss_planetary.lat(2000:4000) = [];
gnss_planetary.h(2000:4000) = [];
gnss_planetary.vel(2000:4000,:) = [];

gnss_planetary_sparse_r = gnss_planetary; % reference
[gnss_planetary,~] = gnss_gen(gnss_planetary_sparse_r, gnss_planetary); % degredation

%% Saving 

save('./canada-planetary-data/data/gnss_planetary_r.mat','gnss_planetary_r');
save('./canada-planetary-data/data/gnss_planetary_sparse_r.mat','gnss_planetary_sparse_r');
save('./canada-planetary-data/data/gnss_sparse_planetary.mat','gnss_planetary');