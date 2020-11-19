function [nav_e] = ins_gnss(imu, gnss, att_mode)
% ins_gnss: loosely-coupled integrated navigation system.
%
% ins_gnss integrates IMU and GNSS measurements by using an Extended Kalman filter.
%
% INPUT
%   imu, IMU data structure.
%         t: Ix1 time vector (seconds).
%        fb: Ix3 accelerations vector in body frame XYZ (m/s^2).
%        wb: Ix3 turn rates vector in body frame XYZ (radians/s).
%       arw: 1x3 angle random walks (rad/s/root-Hz).
%       vrw: 1x3 velocity random walks (m/s^2/root-Hz).
%      gstd: 1x3 gyros standard deviations (radians/s).
%      astd: 1x3 accrs standard deviations (m/s^2).
%    gb_sta: 1x3 gyros static biases or turn-on biases (radians/s).
%    ab_sta: 1x3 accrs static biases or turn-on biases (m/s^2).
%    gb_dyn: 1x3 gyros dynamic biases or bias instabilities (radians/s).
%    ab_dyn: 1x3 accrs dynamic biases or bias instabilities (m/s^2).
%   gb_corr: 1x3 gyros correlation times (seconds).
%   ab_corr: 1x3 accrs correlation times (seconds).
%    gb_psd: 1x3 gyros dynamic biases PSD (rad/s/root-Hz).
%    ab_psd: 1x3 accrs dynamic biases PSD (m/s^2/root-Hz);
%      freq: 1x1 sampling frequency (Hz).
% ini_align: 1x3 initial attitude at t(1).
% ini_align_err: 1x3 initial attitude errors at t(1).
%
%	gnss, GNSS data structure.
%         t: Mx1 time vector (seconds).
%       lat: Mx1 latitude (radians).
%       lon: Mx1 longitude (radians).
%         h: Mx1 altitude (m).
%       vel: Mx3 NED velocities (m/s).
%       std: 1x3 position standard deviations (rad, rad, m).
%      stdm: 1x3 position standard deviations (m, m, m).
%      stdv: 1x3 velocity standard deviations (m/s).
%      larm: 3x1 lever arm from IMU to GNSS antenna (x-fwd, y-right, z-down) (m).
%      freq: 1x1 sampling frequency (Hz).
%   zupt_th: 1x1 ZUPT threshold (m/s).
%  zupt_win: 1x1 ZUPT time window (seconds).
%       eps: 1x1 time interval to compare current IMU time to current GNSS time vector (s).
%
%  att_mode: attitude mode string.
%      'quaternion': attitude updated in quaternion format. Default value.
%             'dcm': attitude updated in Direct Cosine Matrix format.
%
% OUTPUT
%   nav_e, INS/GNSS navigation estimates data structure.
%         t: Ix1 INS time vector (seconds).
%        tg: Mx1 GNSS time vector, when Kalman filter was executed (seconds).
%      roll: Ix1 roll (radians).
%     pitch: Ix1 pitch (radians).
%       yaw: Ix1 yaw (radians).
%       vel: Ix3 NED velocities (m/s).
%       lat: Ix1 latitude (radians).
%       lon: Ix1 longitude (radians).
%         h: Ix1 altitude (m).
%        xi: Mx15 Kalman filter a priori states.
%        xp: Mx15 Kalman filter a posteriori states.
%         z: Mx6  INS/GNSS measurements
%         v: Mx6  Kalman filter innovations.
%         b: Mx6 Kalman filter biases compensations, [gb_dyn ab_dyn].
%         A: Mx225 Kalman filter transition-state matrices, one matrix per
%          row ordered by columns.
%        Pp: Mx225 Kalman filter a posteriori covariance matrices, one
%         matrix per row ordered by columns.
%        Pi: Mx225 Kalman filter a priori covariance matrices, one matrix
%         per row ordered by columns.
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
%   R. Gonzalez, J. Giribet, and H. Patiño. NaveGo: a
% simulation framework for low-cost integrated navigation systems,
% Journal of Control Engineering and Applied Informatics, vol. 17,
% issue 2, pp. 110-120, 2015. Alg. 2.
%
%   ZUPT algothim based on Paul Groves, Principles of GNSS, Inertial, and
% Multisensor Integrated Navigation Systems (2008). Chapter 13: INS
% Alignment and Zero Velocity Updates.
%
%   ins_gps.m, ins_gnss function is based on that previous NaveGo function.
%
% Version: 007
% Date:    2020/10/05
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

if nargin < 3, att_mode  = 'quaternion'; end

%% ZUPT detection algorithm

zupt = false;

%% PREALLOCATION

% Constant matrices
I = eye(3);
O = zeros(3);

% Length of INS time vector
LI = length(imu.t);

% Length of GNSS time vector
LG = length(gnss.t);

% Preallocation of attitude vectors
roll_e  = zeros (LI, 1);
pitch_e = zeros (LI, 1);
yaw_e   = zeros (LI, 1);
%  yawm_e  = zeros (Mi, 1);

% Initial attitude at INS time = 1
roll_e(1)  = imu.ini_align(1);
pitch_e(1) = imu.ini_align(2);
yaw_e(1)   = imu.ini_align(3);
%     yawm_e(1)  = imu.ini_align(3);
DCMnb = euler2dcm([roll_e(1); pitch_e(1); yaw_e(1);]);
DCMbn = DCMnb';
qua   = euler2qua([roll_e(1) pitch_e(1) yaw_e(1)]);

% Preallocation of velocity vector
vel_e   = zeros (LI, 3);

% Initial velocity at INS time = 1
vel_e(1,:) = gnss.vel(1,:);

% Preallocation of position vectors
lat_e    = zeros (LI,1);
lon_e    = zeros (LI,1);
h_e      = zeros (LI, 1);

% Initial position at INS time = 1
h_e(1)   = gnss.h(1);
lat_e(1) = gnss.lat(1);
lon_e(1) = gnss.lon(1);

% Biases
gb_dyn = imu.gb_dyn';
ab_dyn = imu.ab_dyn';

% Initialize Kalman filter matrices

% Prior estimates
kf.xi = [ zeros(1,9), imu.gb_dyn, imu.ab_dyn ]';  % Error vector state
kf.Pi = diag([imu.ini_align_err, gnss.stdv, gnss.std, imu.gb_dyn, imu.ab_dyn].^2);

kf.Q  = diag([imu.arw, imu.vrw, imu.gb_psd, imu.ab_psd].^2);

fb_corrected = (imu.fb(1,:)' + ab_dyn );
f_n = (DCMbn * fb_corrected);

% Vector to update matrix F
upd = [gnss.vel(1,:) gnss.lat(1) gnss.h(1) f_n'];

% Update matrices F and G
[kf.F, kf.G] = F_update(upd, DCMbn, imu);

[RM,RN] = radius(gnss.lat(1));
Tpr = diag([(RM + gnss.h(1)), (RN + gnss.h(1)) * cos(gnss.lat(1)), -1]);  % radians-to-meters
        
% Update matrix H
kf.H = [ O I O O O ;
    O O Tpr O O ; ];
kf.R = diag([gnss.stdv gnss.stdm]).^2;
kf.z = [ gnss.stdv, gnss.stdm ]';
        
% Propagate prior estimates to get xp(1) and Pp(1)
kf = kf_update( kf );

% PENDING: UD filter matrices
% [Up, Dp] = myUD(S.P);
% dp = diag(Dp);

% DEC = 0.5 * 180/pi;             % Magnetic declination (radians)

% Preallocation of Kalman filter matrices for later performance analysis
xi = zeros(LG, 15);        % Evolution of Kalman filter a priori states, xi
xp = zeros(LG, 15);        % Evolution of Kalman filter a posteriori states, xp
z = zeros(LG, 6);          % INS/GNSS measurements
v = zeros(LG, 6);          % Kalman filter innovations
b = zeros(LG, 6);          % Biases compensantions after Kalman filter correction

A  = zeros(LG, 225);       % Transition-state matrices, A
Pi = zeros(LG, 225);       % A priori covariance matrices, Pi
Pp = zeros(LG, 225);       % A posteriori covariance matrices, Pp
K  = zeros(LG, 90);        % Kalman gain matrices, K
S  = zeros(LG, 36);        % Innovation matrices, S
ob = zeros(LG, 1);         % Number of observable states at each GNSS data arriving

% Initial matrices for Kalman filter performance analysis
xp(1,:) = kf.xp';
Pp(1,:) = reshape(kf.Pp, 1, 225);
b(1,:)  = [imu.gb_sta, imu.ab_sta];

% INS (IMU) time is the master clock
for i = 2:LI
    
    %% INERTIAL NAVIGATION SYSTEM (INS)
    
    % Print a dot on console every 10,000 INS executions
    if (mod(i,10000) == 0), fprintf('. ');  end
    % Print a return on console every 200,000 INS executions
    if (mod(i,200000) == 0), fprintf('\n'); end
    
    % IMU sampling interval
    dti = imu.t(i) - imu.t(i-1);
    
    % Inertial sensors corrected with KF biases estimation
    wb_corrected = (imu.wb(i,:)' + gb_dyn );
    fb_corrected = (imu.fb(i,:)' + ab_dyn );
    
    % Turn-rates update
    omega_ie_n = earthrate(lat_e(i-1));
    omega_en_n = transportrate(lat_e(i-1), vel_e(i-1,1), vel_e(i-1,2), h_e(i-1));
    
    % Attitude update
    [qua_n, DCMbn, euler] = att_update(wb_corrected, DCMbn, qua, ...
        omega_ie_n, omega_en_n, dti, att_mode);
    roll_e(i) = euler(1);
    pitch_e(i)= euler(2);
    yaw_e(i)  = euler(3);
    qua = qua_n;
    
    % Gravity update
    g_n = gravity(lat_e(i-1), h_e(i-1));
    
    % Velocity update
    f_n = (DCMbn * fb_corrected);
    vel_n = vel_update(f_n, vel_e(i-1,:), omega_ie_n, omega_en_n, g_n', dti);
    vel_e (i,:) = vel_n;
    
    % Position update
    pos_n = pos_update([lat_e(i-1) lon_e(i-1) h_e(i-1)], vel_e(i,:), dti);
    lat_e(i) = pos_n(1);
    lon_e(i) = pos_n(2);
    h_e(i)   = pos_n(3);
    
    % PENDING. Magnetic heading update
    %         yawm_e(i) = hd_update (imu.mb(i,:), roll_e(i),  pitch_e(i), D);
    
    % ZUPT detection algorithm
    idz = floor( gnss.zupt_win / dti ); % Index to set ZUPT window time
    
    if ( i > idz )
        
        vel_m = mean (vel_e(i-idz:i , :));
        
        if (abs(vel_m) <= gnss.zupt_th)
            
            % Alternative attitude ZUPT correction
            % roll_e(i) = (roll_e(i-idz , :));
            % pitch_e(i)= (pitch_e(i-idz , :));
            % yaw_e(i)  = (yaw_e(i-idz, :));
            
            roll_e(i) = mean (roll_e(i-idz:i , :));
            pitch_e(i)= mean (pitch_e(i-idz:i , :));
            yaw_e(i)  = mean (yaw_e(i-idz:i , :));
            
            lat_e(i) = mean (lat_e(i-idz:i , :));
            lon_e(i) = mean (lon_e(i-idz:i , :));
            h_e(i)   = mean (h_e(i-idz:i , :));
            
            zupt = true;

        end
    end
    
    %% KALMAN FILTER UPDATE
    
    % Check if there is new GNSS measurement to process at current INS time
    gdx =  find (gnss.t >= (imu.t(i) - gnss.eps) & gnss.t < (imu.t(i) + gnss.eps));
    
    if ( ~isempty(gdx) && gdx > 1)
        
%         gdx   % DEBUG
        
        %% MEASUREMENTS
        
        % Update meridian and normal radii of curvature
        [RM,RN] = radius(lat_e(i));
        
        % Radians-to-meters matrix
        Tpr = diag([(RM + h_e(i)), (RN + h_e(i)) * cos(lat_e(i)), -1]);  
        
        % Measurements for position in meters with lever arm correction
        zp = Tpr * ([lat_e(i); lon_e(i); h_e(i);] - [gnss.lat(gdx); gnss.lon(gdx); gnss.h(gdx);]) ...
            + (DCMbn * gnss.larm);
        
        % Innovations for velocity with lever arm correction
        zv = (vel_e(i,:) - gnss.vel(gdx,:) - ((omega_ie_n + omega_en_n) .* (DCMbn * gnss.larm))' ...
            + (DCMbn * skewm(wb_corrected) * gnss.larm )' )';
        
        %% KALMAN FILTER
        
        % GNSS sampling interval
        dtg = gnss.t(gdx) - gnss.t(gdx-1);
        
        % Vector to update matrix F
        upd = [vel_e(i,:) lat_e(i) h_e(i) f_n'];
        
        % Update matrices F and G
        [kf.F, kf.G] = F_update(upd, DCMbn, imu);
        
        % Update matrix H
        if(zupt == false)
            kf.H = [ O I O O O ;
                O O Tpr O O ; ];
            kf.R = diag([gnss.stdv gnss.stdm]).^2;
            kf.z = [ zv' zp' ]';
        else
            kf.H = [ O I O O O ; ];
            kf.R = diag([gnss.stdv]).^2;
            kf.z = zv;
        end
        
        % Execute the extended Kalman filter
        kf.xp(1:9) = 0;              % states 1:9 are forced to be zero (error-state approach)
        kf = kalman(kf, dtg);
        
        %% OBSERVABILITY
        
        % Number the observable states at current GNSS time
        ob(gdx) = rank(obsv(kf.F, kf.H)); 
                
        %% INS/GNSS CORRECTIONS
        
        % Quaternion corrections
        % Crassidis. Eq. 7.34 and A.174a.
        antm = [0 qua_n(3) -qua_n(2); -qua_n(3) 0 qua_n(1); qua_n(2) -qua_n(1) 0];
        qua = qua_n + 0.5 .* [qua_n(4)*eye(3) + antm; -1.*[qua_n(1) qua_n(2) qua_n(3)]] * kf.xp(1:3);
        qua = qua / norm(qua);       % Brute-force normalization
        
        % DCM correction
        DCMbn = qua2dcm(qua);
        
        % Another possible attitude correction algorithm
        %     euler = qua2euler(qua);
        %     roll_e(i) = euler(1);
        %     pitch_e(i)= euler(2);
        %     yaw_e(i)  = euler(3);
        %
        %     E = skewm(S.xp(1:3));
        %     DCMbn = (eye(3) + E) * DCMbn_n;
        
        % Attitude correction
        roll_e(i)  = roll_e(i)  - kf.xp(1);
        pitch_e(i) = pitch_e(i) - kf.xp(2);
        yaw_e(i)   = yaw_e(i)   - kf.xp(3);
        
        % Velocity correction
        vel_e (i,1) = vel_e (i,1) - kf.xp(4);
        vel_e (i,2) = vel_e (i,2) - kf.xp(5);
        vel_e (i,3) = vel_e (i,3) - kf.xp(6);
        
        % Position correction
        lat_e(i) = lat_e(i) - kf.xp(7);
        lon_e(i) = lon_e(i) - kf.xp(8);
        h_e(i)   = h_e(i)   - kf.xp(9);
        
        % Biases estimation
        gb_dyn   = kf.xp(10:12);
        ab_dyn   = kf.xp(13:15);
        
        % Matrices for later Kalman filter performance analysis
        xi(gdx,:) = kf.xi';
        xp(gdx,:) = kf.xp';
        b(gdx,:) = [gb_dyn', ab_dyn'];
        A(gdx,:)  = reshape(kf.A, 1, 225);
        Pi(gdx,:) = reshape(kf.Pi, 1, 225);
        Pp(gdx,:) = reshape(kf.Pp, 1, 225);        
              
        if(zupt == false)
            v(gdx,:)  = kf.v';
            K(gdx,:)  = reshape(kf.K, 1, 90);
            S(gdx,:)  = reshape(kf.S, 1, 36);
        else
            zupt = false;
            v(gdx,:)  = [ kf.v' 0 0 0 ]';
            K(gdx,1:45)  = reshape(kf.K, 1, 45);
            S(gdx,1:9)  = reshape(kf.S, 1, 9);
        end
    end
end

%% Summary from INS/GNSS integration

nav_e.t     = imu.t(1:i, :);    % INS time vector
nav_e.tg    = gnss.t;           % GNSS time vector, which is the time vector when the Kalman filter was executed
nav_e.roll  = roll_e(1:i, :);   % Roll
nav_e.pitch = pitch_e(1:i, :);  % Pitch
nav_e.yaw   = yaw_e(1:i, :);    % Yaw
% nav_e.yawm  = yawm_e(1:i, :);    % Magnetic heading
nav_e.vel   = vel_e(1:i, :);    % NED velocities
nav_e.lat   = lat_e(1:i, :);    % Latitude
nav_e.lon   = lon_e(1:i, :);    % Longitude
nav_e.h     = h_e(1:i, :);      % Altitude

nav_e.xi    = xi;       % A priori states
nav_e.xp    = xp;       % A posteriori states
nav_e.m     = z;        % INS/GNSS measurements
nav_e.v     = v;        % Kalman filter innovations
nav_e.b     = b;        % Biases compensations

nav_e.A     = A;        % Transition matrices
nav_e.Pi    = Pi;       % A priori covariance matrices
nav_e.Pp    = Pp;       % A posteriori covariance matrices
nav_e.K     = K;        % Kalman gain matrices
nav_e.S     = S;        % Innovation matrices
nav_e.ob    = ob;       % Number of observable states at each GNSS data arriving

fprintf('\n');

end
