function  S = kalman(S, dt)
% kalman: Kalman filter algorithm.
%
% INPUT:
%   S, data structure with at least the following fields:
%       xp: 21x1 a posteriori state vector (old).
%        z: 6x1 innovations vector.
%        F: 21x21 state transition matrix.
%        H: 6x21 observation matrix.
%        Q: 12x12 process noise covariance.
%        R: 6x6  observation noise covariance.
%       Pp: 21x21 a posteriori error covariance.
%        G: 21x12 control-input matrix.      
%   		dt: sampling interval. 
%
% OUTPUT:
%    S, the following fields are updated:
%       xi: 21x1 a priori state vector (updated).
%       xp: 21x1 a posteriori state vector (updated).
%		 		 v: 6x1 innovation vector. 
%        A: 21x21 state transition matrix.
%        K: 21x6  Kalman gain matrix.
%       Qd: 21x6  discrete process noise covariance.
%       Pi: 21x21 a priori error covariance.
%       Pp: 21x21 a posteriori error covariance.  
%        S: 6x6  innovation (or residual) covariance.
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
%       R. Gonzalez, J. Giribet, and H. Patiño. NaveGo: a
% simulation framework for low-cost integrated navigation systems,
% Journal of Control Engineering and Applied Informatics, vol. 17,
% issue 2, pp. 110-120, 2015. Alg. 1.
%
%       Dan Simon. Optimal State Estimation. Chapter 5. John Wiley 
%     Dan Simon. Optimal State Estimation. Chapter 5. John Wiley 
% & Sons. 2006.   
%
% Version: 006
% Date:    2019/03/15
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

I = eye(max(size(S.F)));

% Discretization of continous-time system
S.A =  expm(S.F * dt);          				% Exact solution for linerar systems
% S.A = I + (S.F * dt);         				% Approximated solution by Euler method 
S.Qd = (S.G * S.Q * S.G') .* dt;                % Digitalized covariance matrix

% **********************************************************************
% PREDICTION STEP
% **********************************************************************

% Step 1, predict the a priori state vector xi
S.xi = S.A * S.xp;

% Step 2, update the a priori covariance matrix Pi
S.Pi = (S.A * S.Pp * S.A') + S.Qd;
S.Pi =  0.5 .* (S.Pi + S.Pi');                  % Force Pi to be symmetric matrix

% **********************************************************************
% UPDATE STEP
% **********************************************************************

% Step 3, update Kalman gain
S.S = (S.R + S.H * S.Pi * S.H');				% Innovations covariance matrix
S.v =  S.z - S.H * S.xi; 						% Innovations vector
S.K = (S.Pp * S.H') * (S.S)^(-1) ;				% Kalman gain matrix

% Step 4, update the a posteriori state vector xp
S.xp = S.xi + S.K * S.v; 

% Step 5, update the a posteriori covariance matrix Pp
% S.Pp = S.Pi - S.K * S.S *  S.K';                
J = (I - S.K * S.H);                          % Joseph stabilized version     
S.Pp = J * S.Pi * J' + S.K * S.R * S.K';      % Alternative implementation
S.Pp =  0.5 .* (S.Pp + S.Pp');                  % Force Pi to be symmetric matrix

end
