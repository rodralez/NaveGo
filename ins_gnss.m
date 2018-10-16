function [nav_e] = ins_gnss(imu, gnss, att_mode)
% ins_gnss: loosely-coupled integrated navigation system.
%
% ins_gnss integrates IMU and GNSS measurements by using an Extended Kalman filter.
%
% INPUT:
%   imu, IMU data structure.
%         t: Ix1 time vector (seconds).
%        fb: Ix3 accelerations vector in body frame XYZ (m/s^2).
%        wb: Ix3 turn rates vector in body frame XYZ (radians/s).
%       arw: 1x3 angle random walks (rad/s/root-Hz).
%       vrw: 1x3 angle velocity walks (m/s^2/root-Hz).
%      gstd: 1x3 gyros standard deviations (radians/s).
%      astd: 1x3 accrs standard deviations (m/s^2).
%    gb_fix: 1x3 gyros static biases or turn-on biases (radians/s).
%    ab_fix: 1x3 accrs static biases or turn-on biases (m/s^2).
%  gb_drift: 1x3 gyros dynamic biases or bias instabilities (radians/s).
%  ab_drift: 1x3 accrs dynamic biases or bias instabilities (m/s^2).
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
%       eps: 1x1 time interval to compare IMU time vector to GNSS time vector (s).      
%
%	att_mode, attitude mode string.
%      'quaternion': attitude updated in quaternion format. Default value.
%             'dcm': attitude updated in Direct Cosine Matrix format.
%
% OUTPUT:
%   nav_e, INS/GNSS navigation estimates data structure.
%         t: Ix1 time vector (seconds).
%        tk: Mx1 time vector when Kalman filter was executed (seconds).
%      roll: Ix1 roll (radians).
%     pitch: Ix1 pitch (radians).
%       yaw: Ix1 yaw (radians).
%       vel: Ix3 NED velocities (m/s).
%       lat: Ix1 latitude (radians).
%       lon: Ix1 longitude (radians).
%         h: Ix1 altitude (m).
%        Pp: Mx441 Kalman filter a posteriori covariance matrices.
%        Pi: Mx441 Kalman filter a priori covariance matrices.
%         A: Mx441 Kalman filter transition-state matrices.
%         B: Mx12 Kalman filter biases compensations.
%        In: Mx6  Kalman filter innovations.
%        Xi: Mx21 Kalman filter a priori states.
%        Xp: Mx21 Kalman filter a posteriori states.
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
% Multisensor Integrated Navigation Systems. CHAPTER 13, INS Alignment
% and Zero Velocity Updates.
%
%   ins_gps.m, ins_gnss function is based on that previous function.
%
% Version: 002
% Date:    2018/10/16
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

if nargin < 3, att_mode  = 'quaternion'; end

%% ZUPT detection algorithm

zupt = false;

%% PREALLOCATION

% Length of INS time vector
LI = length(imu.t);

% Length of GNSS time vector
LG = length(gnss.t);

% Attitude
roll_e  = zeros (LI, 1);
pitch_e = zeros (LI, 1);
yaw_e   = zeros (LI, 1);
%     yawm_e  = zeros (Mi, 1);
% Initialize estimates at INS time = 1
roll_e(1)  = imu.ini_align(1);
pitch_e(1) = imu.ini_align(2);
yaw_e(1)   = imu.ini_align(3);
%     yawm_e(1)  = imu.ini_align(3);

DCMnb = euler2dcm([roll_e(1); pitch_e(1); yaw_e(1);]);
DCMbn = DCMnb';
qua   = euler2qua([roll_e(1) pitch_e(1) yaw_e(1)]);

% Velocities
vel_e   = zeros (LI, 3);
vel_e(1,:) = gnss.vel(1,:);

% Positions
lat_e    = zeros (LI,1);
lon_e    = zeros (LI,1);
h_e      = zeros (LI, 1);
h_e(1)   = gnss.h(1);
lat_e(1) = gnss.lat(1);
lon_e(1) = gnss.lon(1);

% Biases
gb_drift = imu.gb_drift';
ab_drift = imu.ab_drift';
gb_fix   = imu.gb_fix';
ab_fix   = imu.ab_fix';

% Initialize Kalman filter matrices
S.xp = [ zeros(1,9), imu.gb_fix, imu.ab_fix, imu.gb_drift, imu.ab_drift ]';  % Error vector state
S.Pp = diag([imu.ini_align_err, gnss.stdv, gnss.std, imu.gb_fix, imu.ab_fix, imu.gb_drift, imu.ab_drift].^2);
S.R  = diag([gnss.stdv, gnss.stdm].^2);
S.Q  = diag([imu.arw, imu.vrw, imu.gb_psd, imu.ab_psd].^2);

% PENDING: UD filter matrices
% [Up, Dp] = myUD(S.P);
% dp = diag(Dp);

% DEC = 0.5 * 180/pi;             % Magnetic declination (radians)

% Kalman matrices for later analysis
In = zeros(LG, 6);         % Kalman filter innovations
Pi = zeros(LG, 441);       % Elements from a priori covariance matrices, Pi
Pp = zeros(LG, 441);       % Elements from a posteriori covariance matrices, Pp
A  = zeros(LG, 441);       % Elements from transition-state matrices, A
Xi = zeros(LG, 21);        % Evolution of Kalman filter a priori states, xi
Xp = zeros(LG, 21);        % Evolution of Kalman filter a posteriori states, xp
B  = zeros(LG, 12);        % Biases compensantions after Kalman filter correction

% Initialize matrices for INS/GNSS performance analysis
Pp(1,:) = reshape(S.Pp, 1, 441);
B(1,:)  = [gb_fix', ab_fix', gb_drift', ab_drift'];
Xp(1,:) = S.xp';

% Constant matrices
I = eye(3);
Z = zeros(3);

% Index for INS/GNSS performance analysis matrices
j = 1;

% IMU time is the master clock
for i = 2:LI    
    
    %% INERTIAL NAVIGATION SYSTEM (INS)
    
    % Print a dot on console every 10,000 INS executions    
    if (mod(i,10000) == 0), fprintf('. ');  end
    % Print a return on console every 200,000 INS executions
    if (mod(i,200000) == 0), fprintf('\n'); end
    
    % INS period
    dti = imu.t(i) - imu.t(i-1);
    
    % Correct inertial sensors
    wb_corrected = (imu.wb(i,:)' + gb_fix + gb_drift );
    fb_corrected = (imu.fb(i,:)' + ab_fix + ab_drift );
    
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
    gn = gravity(lat_e(i-1), h_e(i-1));
    
    % Velocity update
    fn = (DCMbn * fb_corrected);
    vel_n = vel_update(fn, vel_e(i-1,:), omega_ie_n, omega_en_n, gn', dti);
    vel_e (i,:) = vel_n;
    
    % Position update
    pos = pos_update([lat_e(i-1) lon_e(i-1) h_e(i-1)], vel_e(i,:), dti);
    lat_e(i) = pos(1);
    lon_e(i) = pos(2);
    h_e(i)   = pos(3);
    
    % PENDING. Magnetic heading update
    %         yawm_e(i) = hd_update (imu.mb(i,:), roll_e(i),  pitch_e(i), D);
    
    % ZUPT detection algorithm
    idz = floor( gnss.zupt_win / dti ); % Index to set ZUPT window time
    
    if ( i > idz )
        
        vel_m = mean (vel_e(i-idz:i , :));
        
        if (abs(vel_m) <= gnss.zupt_th)
            
            %                 Alternative attitude ZUPT correction
            %                 roll_e(i) = (roll_e(i-idz , :));
            %                 pitch_e(i)= (pitch_e(i-idz , :));
            %                 yaw_e(i)  = (yaw_e(i-idz, :));
            
            roll_e(i) = mean (roll_e(i-idz:i , :));
            pitch_e(i)= mean (pitch_e(i-idz:i , :));
            yaw_e(i)  = mean (yaw_e(i-idz:i , :));
            
            lat_e(i) = mean (lat_e(i-idz:i , :));
            lon_e(i) = mean (lon_e(i-idz:i , :));
            h_e(i)   = mean (h_e(i-idz:i , :));
            
            zupt = true;
        else
            zupt = false;
        end
        
    end
    
    %% KALMAN FILTER UPDATE 
    
    gdx =  find (gnss.t >= imu.t(i) - gnss.eps & gnss.t < imu.t(i) + gnss.eps);
    
    if ( ~isempty(gdx) )
        
        %% INNOVATIONS
        
        [RM,RN] = radius(lat_e(i));
        Tpr = diag([(RM + h_e(i)), (RN + h_e(i)) * cos(lat_e(i)), -1]);  % radians-to-meters
        
        % Innovations
        % Lever arm correction for position
        zp = Tpr * ([lat_e(i); lon_e(i); h_e(i);] - [gnss.lat(gdx); gnss.lon(gdx); gnss.h(gdx);]) ...
            + (DCMbn * gnss.larm);
        
        % Lever arm corrections for velocity
        zv = (vel_e(i,:) - gnss.vel(gdx,:) - ((omega_ie_n + omega_en_n) .* (DCMbn * gnss.larm))' ...
            + (DCMbn * skewm(wb_corrected) * gnss.larm )' )';
        
        %% KALMAN FILTER
        
        % GNSS period
        dtg = gnss.t(gdx) - gnss.t(gdx-1);
        
        % Vector to update matrix F
        upd = [vel_e(i,:) lat_e(i) h_e(i) fn'];
        
        % Update matrices F and G
        [S.F, S.G] = F_update(upd, DCMbn, imu);
        
        % Update matrix H
        if(zupt == false)
            S.H = [ Z I Z Z Z Z Z;
                Z Z Tpr Z Z Z Z; ];
            S.R = diag([gnss.stdv gnss.stdm]).^2;
            S.z = [ zv' zp' ]';
        else
            S.H = [ Z I Z Z Z Z Z; ];
            S.R = diag([gnss.stdv]).^2;
            S.z = zv;
        end
        
        % Execute the extended Kalman filter
        S.xp(1:9) = zeros(9,1);     % states 1:9 are forced to zero (error-state approach)
        S = kalman(S, dtg);
        
        %% INS/GNSS CORRECTIONS
        
        % Quaternion corrections
        % Crassidis. Eq. 7.34 and A.174a.
        antm = [0 qua_n(3) -qua_n(2); -qua_n(3) 0 qua_n(1); qua_n(2) -qua_n(1) 0];
        qua = qua_n + 0.5 .* [qua_n(4)*eye(3) + antm; -1.*[qua_n(1) qua_n(2) qua_n(3)]] * S.xp(1:3);
        qua = qua / norm(qua);       % Brute-force normalization
        
        % DCM correction
        DCMbn = qua2dcm(qua);
        
        % Another possible attitude correction
        %     euler = qua2euler(qua);
        %     roll_e(i) = euler(1);
        %     pitch_e(i)= euler(2);
        %     yaw_e(i)  = euler(3);
        %
        %     E = skewm(S.xp(1:3));
        %     DCMbn = (eye(3) + E) * DCMbn_n;
        
        % Attitude corrections
        roll_e(i)  = roll_e(i)  - S.xp(1);
        pitch_e(i) = pitch_e(i) - S.xp(2);
        yaw_e(i)   = yaw_e(i)   - S.xp(3);
        
        % Velocity corrections
        vel_e (i,1) = vel_e (i,1) - S.xp(4);
        vel_e (i,2) = vel_e (i,2) - S.xp(5);
        vel_e (i,3) = vel_e (i,3) - S.xp(6);
        
        % Position corrections
        lat_e(i) = lat_e(i) - double(S.xp(7));
        lon_e(i) = lon_e(i) - double(S.xp(8));
        h_e(i)   = h_e(i)   - S.xp(9);
        
        % Biases corrections
        gb_fix   = S.xp(10:12);
        ab_fix   = S.xp(13:15);
        gb_drift = S.xp(16:18);
        ab_drift = S.xp(19:21);
        
        % Matrices for later INS/GNSS performance analysis
        Xi(j,:) = S.xi';
        Xp(j,:) = S.xp';
        Pi(j,:) = reshape(S.Pi, 1, 441);
        Pp(j,:) = reshape(S.Pp, 1, 441);
        A(j,:)  = reshape(S.A, 1, 441);
        if(zupt == true)
            In(j,:) = [ zv; zeros(3,1);]';
        else
            In(j,:) = S.z';
        end
        B(j,:) = [gb_fix', ab_fix', gb_drift', ab_drift'];
        
        j = j + 1;
        
    end
end

%% Estimates from INS/GNSS integration

nav_e.t     = imu.t(1:i, :);    % IMU time vector
nav_e.tg    = gnss.t;           % GNSS time vector, also time vector when the Kalman filter was executed
nav_e.roll  = roll_e(1:i, :);   % Roll
nav_e.pitch = pitch_e(1:i, :);  % Pitch
nav_e.yaw   = yaw_e(1:i, :);    % Yaw
% nav_e.yawm  = yawm_e(1:i, :);    % Magnetic heading
nav_e.vel   = vel_e(1:i, :);    % NED velocities
nav_e.lat   = lat_e(1:i, :);    % Latitude
nav_e.lon   = lon_e(1:i, :);    % Longitude
nav_e.h     = h_e(1:i, :);      % Altitude
nav_e.Pi    = Pi;       % Pi matrices
nav_e.Pp    = Pp;       % Pp matrices
nav_e.A     = A;        % A matrices
nav_e.B     = B;        % Kalman filter biases compensations
nav_e.In    = In;       % Kalman filter innovations
nav_e.Xi    = Xi;       % Kalman filter a priori states
nav_e.Xp    = Xp;       % Kalman filter a posteriori states

fprintf('\n');

end
