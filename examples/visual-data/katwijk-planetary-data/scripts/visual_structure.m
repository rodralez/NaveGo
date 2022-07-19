% Johann Diep (johann.diep@esa.int) - August 2021
%
% This script builds the structure for the visual dataset.

%% Conversion Constants

G =  9.80665;       % Gravity constant, m/s^2
G2MSS = G;          % g to m/s^2
MSS2G = (1/G);      % m/s^2 to g

D2R = (pi/180);     % degrees to radians
R2D = (180/pi);     % radians to degrees

KT2MS = 0.514444;   % knot to m/s
MS2KMH = 3.6;       % m/s to km/h

%% OpenVINS PLANETARY DATASET ERROR PROFILE

load('./katwijk-planetary-data/data/gnss_planetary_r.mat'); % loading GNSS struct
load('./katwijk-planetary-data/data/imu_planetary.mat'); % loading inertial struct

visual_data_planetary_pose = readtable('./external-data/katwijk/pose_data.csv'); % read timestamps, position and attitude data
visual_data_planetary_twist = readtable('./external-data/katwijk/twist_data.csv'); % read linear and angular velocity data
visual_data_planetary_posecov = readtable('./external-data/katwijk/posecov_data.csv'); % read position and attitude covariance data
visual_data_planetary_twistcov = readtable('./external-data/katwijk/twistcov_data.csv'); % read linear and angular velocity covariance data

%   visual, visual data structure
%         t: Vx1 time vector (seconds)
%       lat: Vx1 latitude (radians).
%       lon: Vx1 longitude (radians).
%         h: Vx1 altitude (m).
%       vel: Vx3 NED velocities (m/s).
%     covvm: Vx36 velocity and position covariance matrices (m^2 and m^2/s^2).
%      covv: Vx9 velocity covariance matrices (m^2/s^2). 
%      larm: 3x1 lever arm from IMU to camera (x-fwd, y-right, z-down) (m).
%      freq: 1x1 sampling frequency (Hz).
%       eps: 1x1 time interval to compare current IMU time to current GNSS time vector (s).

% parsing the timestamps of the measurements
Seconds = table2array(visual_data_planetary_pose(:,4));
Nanoseconds = table2array(visual_data_planetary_pose(:,5));
Timestamps = Seconds + Nanoseconds/10^(9);
visual_planetary.t = Timestamps - Timestamps(1);

% position and velocity preprocessing
Positions = table2array(visual_data_planetary_pose(:,6:7));
Heights = table2array(visual_data_planetary_pose(:,8));
%RotationAngle = deg2rad(-70);
RotationAngle = deg2rad(-110);
RotationMatrix = [cos(RotationAngle),-sin(RotationAngle);sin(RotationAngle),cos(RotationAngle)];
Positions_rotated = RotationMatrix * Positions';
[visual_planetary.lat, visual_planetary.lon, visual_planetary.h] = ned2geodetic(Positions_rotated(2,:)',Positions_rotated(1,:)',-Heights, ...
    gnss_planetary_r.lat(1),gnss_planetary_r.lon(1),gnss_planetary_r.h(1),wgs84Ellipsoid,'radians');

Velocities = table2array(visual_data_planetary_twist(:,1:2));
Velocities_rotated = RotationMatrix * Velocities';
visual_planetary.vel(:,1:2) = Velocities_rotated';
visual_planetary.vel(:,3) = -table2array(visual_data_planetary_twist(:,3));

% parsing the covariances
for i = 1:size(table2array(visual_data_planetary_posecov),1)
        PoseCovariance = table2array(visual_data_planetary_posecov(i,:));
        TwistCovariance = table2array(visual_data_planetary_twistcov(i,:));
        
        PoseCovariance = reshape(PoseCovariance,[6,6])';
        TwistCovariance = reshape(TwistCovariance,[6,6])';
        
        PositionCovariance = PoseCovariance(1:3,1:3);   
        VelocityCovariance = TwistCovariance(1:3,1:3);       
        
        IntermediateCovariance = [VelocityCovariance,zeros(3,3);zeros(3,3),PositionCovariance];
        
        visual_planetary.covv(i,:) = reshape(VelocityCovariance',[1,9]);
        visual_planetary.covvm(i,:) = reshape(IntermediateCovariance',[1,36]);       
end

visual_planetary.larm = [0,0,0]'; % no lever arm needed
%visual_planetary.eps = mean(diff(imu_planetary.t))*12; % rule of thumb for choosing eps
visual_planetary.eps = mean(diff(imu_planetary.t))/3; % rule of thumb for choosing eps

% downsample
visual_planetary.t = downsample(visual_planetary.t,20);
visual_planetary.lat = downsample(visual_planetary.lat,20);
visual_planetary.lon = downsample(visual_planetary.lon,20);
visual_planetary.h = downsample(visual_planetary.h,20);
visual_planetary.vel = downsample(visual_planetary.vel,20);
visual_planetary.covv = downsample(visual_planetary.covv,20);
visual_planetary.covvm = downsample(visual_planetary.covvm,20);

visual_planetary.freq = 1/mean(diff(visual_planetary.t)); % estimating the frequency

%save('../data/visual_planetary.mat','visual_planetary');
save('./katwijk-planetary-data/data/visual_planetary.mat','visual_planetary');