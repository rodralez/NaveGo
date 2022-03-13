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

load('gnss_planetary_r.mat');
load('imu_planetary.mat');

visual_data_planetary_pose = readtable('Data/Canada/pose_data.csv');
visual_data_planetary_twist = readtable('Data/Canada/twist_data.csv');
visual_data_planetary_posecov = readtable('Data/Canada/posecov_data.csv');
visual_data_planetary_twistcov = readtable('Data/Canada/twistcov_data.csv');

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

Cutoff = 1500;

% parsing the timestamps of the measurements
Seconds = table2array(visual_data_planetary_pose(Cutoff:end,4));
Nanoseconds = table2array(visual_data_planetary_pose(Cutoff:end,5));
Timestamps = Seconds + Nanoseconds/10^(9);
visual_planetary.t = Timestamps - Timestamps(1);

% position and velocity preprocessing
Positions = table2array(visual_data_planetary_pose(Cutoff:end,6:7))-table2array(visual_data_planetary_pose(Cutoff,6:7));
Heights = table2array(visual_data_planetary_pose(Cutoff:end,8))-table2array(visual_data_planetary_pose(Cutoff,8));
RotationAngle = deg2rad(-100);
RotationMatrix = [cos(RotationAngle),-sin(RotationAngle);sin(RotationAngle),cos(RotationAngle)];
Positions_rotated = RotationMatrix * Positions';

[visual_planetary.lat, visual_planetary.lon, visual_planetary.h] = ned2geodetic(Positions_rotated(2,:)',Positions_rotated(1,:)',-Heights, ...
    gnss_planetary_r.lat(1),gnss_planetary_r.lon(1),gnss_planetary_r.h(1),wgs84Ellipsoid,'radians');

Velocities = table2array(visual_data_planetary_twist(Cutoff:end,1:2));
Velocities_rotated = RotationMatrix * Velocities';
visual_planetary.vel(:,1:2) = Velocities_rotated';
visual_planetary.vel(:,3) = -table2array(visual_data_planetary_twist(Cutoff:end,3));

% parsing the covariances
Count = 1;
for i = Cutoff:size(table2array(visual_data_planetary_posecov),1)
        PoseCovariance = table2array(visual_data_planetary_posecov(i,:));
        TwistCovariance = table2array(visual_data_planetary_twistcov(i,:));
        
        PoseCovariance = reshape(PoseCovariance,[6,6])';
        TwistCovariance = reshape(TwistCovariance,[6,6])';
        
        PositionCovariance = PoseCovariance(1:3,1:3);   
        VelocityCovariance = TwistCovariance(1:3,1:3);       
        
        % [~,Std] = corrcov(PositionCovariance);
        % Std_x(i) = Std(1);
        % Std_y(i) = Std(2);
        % Std_z(i) = Std(3);
        
        IntermediateCovariance = [VelocityCovariance,zeros(3,3);zeros(3,3),PositionCovariance];
        
        visual_planetary.covv(Count,:) = reshape(VelocityCovariance',[1,9]);
        visual_planetary.covvm(Count,:) = reshape(IntermediateCovariance',[1,36]);  
        Count = Count + 1;
end

visual_planetary.larm = [0,0,0]'; % no lever arm needed in case of OpenVINS
visual_planetary.eps = mean(diff(imu_planetary.t))*20; % rule of thumb for choosing eps

% downsample
visual_planetary.t = downsample(visual_planetary.t,33);
visual_planetary.lat = downsample(visual_planetary.lat,33);
visual_planetary.lon = downsample(visual_planetary.lon,33);
visual_planetary.h = downsample(visual_planetary.h,33);
visual_planetary.vel = downsample(visual_planetary.vel,33);
visual_planetary.covv = downsample(visual_planetary.covv,33);
visual_planetary.covvm = downsample(visual_planetary.covvm,33);

visual_planetary.freq = 1/mean(diff(visual_planetary.t)); % estimating the frequency

save('NaveGo-master/examples/planetary-data/canada-planetary-data/data/visual_planetary.mat','visual_planetary');