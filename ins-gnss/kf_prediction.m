function  kf = kf_prediction(kf, dt)
% kf_prediction: Prediction update part of the Kalman filter algorithm.
%
% INPUT
%   kf, data structure with at least the following fields:
%       xp: 15x1 a posteriori state vector (old).
%       Pp: 15x15 a posteriori error covariance matrix (old).
%        F: 15x15 state transition matrix.
%        Q: 12x12 process noise covariance matrix.
%        G: 15x12 control-input matrix.      
%   	dt: sampling interval. 
%
% OUTPUT
%   kf, the following fields are updated:
%       xi: 15x1 a priori state vector (updated).
%       Pi: 15x15 a priori error covariance matrix.
%        A: 15x15 state transition matrix.
%       Qd: 15x6 discrete process noise covariance matrix.
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
%   R. Gonzalez, J. Giribet, and H. Patiño. NaveGo: a
% simulation framework for low-cost integrated navigation systems,
% Journal of Control Engineering and Applied Informatics, vol. 17,
% issue 2, pp. 110-120, 2015. Alg. 1.
%
%   Dan Simon. Optimal State Estimation. Chapter 5. John Wiley 
% & Sons. 2006.   
%
% Version: 001
% Date:    2019/04/19
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

% Discretization of continous-time system
kf.A =  expm(kf.F * dt);          				% Exact solution for linear systems
% S.A = I + (S.F * dt);         				% Approximated solution by Euler method 
kf.Qd = (kf.G * kf.Q * kf.G') .* dt;            % Digitalized covariance matrix

% Step 1, predict the a priori state vector xi
kf.xi = kf.A * kf.xp;

% Step 2, update the a priori covariance matrix Pi
kf.Pi = (kf.A * kf.Pp * kf.A') + kf.Qd;
kf.Pi =  0.5 .* (kf.Pi + kf.Pi');               % Force Pi to be symmetric matrix

end
