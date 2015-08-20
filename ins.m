function [estimates, wb_fix, fb_fix] = ins(imu, gps, ref, precision)
% ins: integrates IMU and GPS measurements using an Extended Kalman filter
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
% Reference: 
%			R. Gonzalez, J. Giribet, and H. Patiño. NaveGo: a 
% simulation framework for low-cost integrated navigation systems, 
% Journal of Control Engineering and Applied Informatics, vol. 17, 
% issue 2, pp. 110-120, 2015. Alg. 2.
%
% Version: 001
% Date:    2015/07/02
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego 

global d2r

if nargin < 4, precision = 'double'; end

tins = imu.t;
tgps = gps.t;

tto = (max(size(tins)));
ttg = (max(size(tgps)));

if strcmp(precision, 'single')

    % Allocate memory for estimates
    roll_e  =  single(zeros (tto,1));
    pitch_e =  single(zeros (tto,1));
    yaw_e   =  single(zeros (tto,1));
    vel_e   =  single(zeros (tto,3));
    h_e     =  single(zeros (tto,1));

    I =  single(eye(3));
    Z =  single(zeros(3));
    Y =  single(zeros(ttg,6));  
    PP = single(zeros(ttg,21));   
    X =  single(zeros(ttg,21));   
    B =  single(zeros(ttg,12));  

    gb_drift = single(imu.gb_drift');
    ab_drift = single(imu.ab_drift');
    gb_fix = single(imu.gb_fix');
    ab_fix = single(imu.ab_fix');
     
    % Initialize
    vel_e(1,:) = single(zeros(1,3));    
    x = single(zeros(21,1));
else
    
    % Allocate memory for estimates
    roll_e  =  (zeros (tto,1));
    pitch_e =  (zeros (tto,1));
    yaw_e   =  (zeros (tto,1));
    vel_e   =  (zeros (tto,3));
    h_e     =  (zeros (tto,1));

    I =  (eye(3));
    Z =  (zeros(3));
    Y =  (zeros(ttg,6));  
    PP = (zeros(ttg,21));   
    X =  (zeros(ttg,21));   
    B =  (zeros(ttg,12));  
    
    gb_drift = imu.gb_drift';
    ab_drift = imu.ab_drift';
    gb_fix = imu.gb_fix';
    ab_fix = imu.ab_fix';
    
    % Initialize    
    vel_e(1,:) = zeros(1,3); 
    x = (zeros(21,1));
end

lat_e   =  (zeros (tto,1));
lon_e   =  (zeros (tto,1));
    
% Initialize estimates at t=1
roll_e (1) = ref.roll(1);
pitch_e(1) = ref.pitch(1);
yaw_e(1)   = ref.yaw(1);
lat_e(1) =   double(gps.lat(1));
lon_e(1) =   double(gps.lon(1));
h_e(1)   =   gps.h(1);

DCMnb_old = euler2dcm([roll_e(1); pitch_e(1); yaw_e(1);]);
DCMbn_old = DCMnb_old';
quaold = euler2qua([roll_e(1); pitch_e(1); yaw_e(1);]);

% Kalman filter matrices
R = diag([gps.stdv gps.stdm].^2);
Q = (diag([imu.arw imu.vrw imu.gpsd imu.apsd ].^2));
P = diag([ [5 5 5 ].*d2r gps.stdv gps.std imu.gb_fix imu.ab_fix imu.gb_drift imu.ab_drift].^2); 
% [Up, Dp] = myUD(P);
% dp = diag(Dp);

PP(1,:) = (diag(P)');
B(1,:)  = [gb_fix', ab_fix', gb_drift', ab_drift'];

% SINS index
i = 2;

wb_fix = zeros(tto,3);
fb_fix = zeros(tto,3);

wb_fix(1,:) = imu.wb(1,:);
fb_fix(1,:) = imu.fb(1,:);

% GPS clock is the master clock
for j = 2:ttg

    while (tins(i) <= tgps(j))
    
        % Print a dot every 10,000 SINS executions
        if (mod(i,10000) == 0), fprintf('. '); end     
        
        % SINS period
        dti = tins(i) - tins(i-1);

        % Correct inertial sensors
        wb_fix(i,:) = (imu.wb(i,:)' + gb_drift + gb_fix);
        fb_fix(i,:) = (imu.fb(i,:)' + ab_drift + ab_fix); 
        
        % Attitude computer
        omega_ie_N = earthrate(lat_e(i-1), precision); 
        omega_en_N = transportrate(lat_e(i-1), vel_e(i-1,1), vel_e(i-1,2), h_e(i-1));
        [quanew, DCMbn_new, ang_v] = att_update(wb_fix(i,:)', DCMbn_old, quaold, ... 
                                                  omega_ie_N, omega_en_N, dti); 
        roll_e(i) = ang_v(1);
        pitch_e(i)= ang_v(2); 
        yaw_e(i)  = ang_v(3); 
        DCMbn_old = DCMbn_new;       
        quaold = quanew;
       
        % Gravity computer
        g = gravity(lat_e(i-1), h_e(i-1));   
        
        % Velocity Computer
        fn = DCMbn_new * (fb_fix(i,:)');
        vel_upd = vel_update(fn, vel_e(i-1,:)', omega_ie_N, omega_en_N, g', dti); %
        vel_e (i,:) = vel_upd';   

        % Position Computer
        pos = pos_update([lat_e(i-1) lon_e(i-1) double(h_e(i-1))], double(vel_e(i,:)), double(dti) );
        lat_e(i)=pos(1); lon_e(i)=pos(2); h_e(i)=single(pos(3));
        
        % Compass Computer        
%         yawm_e(i) = ( hd_update (imu.mb(i,:), roll_e(i),  pitch_e(i), D) );
        
        % Index
        i = i + 1;
    end
    
    % GPS period
    dtg = tgps(j) - tgps(j-1);    
    
    % Vector to update matrix F
    upd = [vel_e(i-1,:) lat_e(i-1) h_e(i-1) fn'];

    [RM,RN] = radius(lat_e(i-1), precision);
    Tpr = diag([(RM+h_e(i-1)), (RN+h_e(i-1))*cos(lat_e(i-1)), -1]);  % rad-to-meters
    
    % Innovations       
    yp = Tpr * ([lat_e(i-1); lon_e(i-1); h_e(i-1);] ... 
        - [gps.lat(j); gps.lon(j); gps.h(j);]) + (DCMbn_new * gps.larm);    

    yv = (vel_e (i-1,:) - gps.vel(j,:))';      
    y = [ yv' yp' ]';
         
    % Update matrices F and G
    [F, G] = F_update(upd, DCMbn_new, imu);
    
    % Update matrix H
    H = [Z I Z Z Z Z Z;
         Z Z Tpr Z Z Z Z; ]; 
        
    % Execute the extended Kalman filter
%     x = [zeros(1,9) gb_fix', ab_fix', gb_drift', ab_drift']';
    
    [xu, P] = kalman(x, y, F, H, G, P, Q, R, dtg);        
%     [xu, Up, dp] = ud_filter(x, y, F, H, G, Q, R, Up, dp, dtg);  
%     P = Up * diag(dp) * Up'; 
      
    X(j,:) = xu';
    PP(j,:) = diag(P)';
    
    % DCM correction
    E = skewm(xu(1:3));    
    DCMbn_old = (eye(3) + E) * DCMbn_new;
    
    % Quaternion correction
    antm = [0 quanew(3) -quanew(2); -quanew(3) 0 quanew(1); quanew(2) -quanew(1) 0];
    quaold = quanew + 0.5 .* [quanew(4)*eye(3) + antm; -1.*[quanew(1) quanew(2) quanew(3)]] * xu(1:3);
    quaold = quaold/norm(quaold);
    
    % Attitude correction
    roll_e(i-1)  = roll_e(i-1) - xu(1);
    pitch_e(i-1) = pitch_e(i-1) - xu(2);
    yaw_e(i-1)   = yaw_e(i-1)  - xu(3);            
    % Velocity correction
    vel_e (i-1,1) = vel_e (i-1,1) - xu(4);
    vel_e (i-1,2) = vel_e (i-1,2) - xu(5);
    vel_e (i-1,3) = vel_e (i-1,3) - xu(6);            
    % Position correction
    lat_e(i-1) = lat_e(i-1) - double(xu(7));
    lon_e(i-1) = lon_e(i-1) - double(xu(8));
    h_e(i-1)   = h_e(i-1)   - xu(9); 
    
    % Biases correction
    gb_fix = gb_fix - xu(10:12); 
    ab_fix = ab_fix - xu(13:15);
    gb_drift = gb_drift - xu(16:18); 
    ab_drift = ab_drift - xu(19:21);
    
    B(j,:) = [gb_fix', ab_fix', gb_drift', ab_drift'];
end 

    estimates.t     = tins(1:i-1, :);
    estimates.roll  = roll_e(1:i-1, :);
    estimates.pitch = pitch_e(1:i-1, :);
    estimates.yaw = yaw_e(1:i-1, :);
    estimates.vel = vel_e(1:i-1, :);
    estimates.lat = lat_e(1:i-1, :);
    estimates.lon = lon_e(1:i-1, :);
    estimates.h   = h_e(1:i-1, :);
    estimates.PP = PP;
    estimates.B  = B;
    estimates.Y  = Y;
    estimates.X  = X;
end
