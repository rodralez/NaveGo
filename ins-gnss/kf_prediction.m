function  kf = kf_prediction(kf, dt)
% kf_prediction: prediction update part of the Kalman filter algorithm.
%
% INPUT
%   kf, data structure with at least the following fields:
%       xp: nx1 a posteriori state vector.
%       Pp: nxn a posteriori error covariance matrix.
%        F: nxn state transition matrix.
%        Q: qxq process noise covariance matrix.
%        G: nxq control-input matrix.      
%   	dt: sampling interval. 
%
% OUTPUT
%   kf, the following fields are updated:
%       xi: nx1 a priori state vector.
%       Pi: nxn a priori error covariance matrix.
%        A: nxn state transition matrix.
%       Qd: nxn discrete process noise covariance matrix.
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
%
%   Simo Särkkä (2013). Bayesian Filtering and Smoothing. Cambridge 
%     University Press.  
%
% Version: 001
% Date:    2022/03/06
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

% Discretization of continous-time system
kf.A =  expm(kf.F * dt);          				% Exact solution for linear systems
% kf.A = I + (kf.F * dt);         				% Approximated solution by Euler method 
kf.Qd = (kf.G * kf.Q * kf.G') .* dt;            % Digitalized covariance matrix

% Step 1, predict the a priori state vector, xi
kf.xi = kf.A * kf.xp;

% Step 2, update the a priori covariance matrix, Pi
kf.Pi = (kf.A * kf.Pp * kf.A') + kf.Qd;
kf.Pi =  0.5 .* (kf.Pi + kf.Pi');               % Force Pi to be a symmetric matrix

end
