function  [xu, S, xp] = kalman(x, z, S, dt)
% kalman: Kalman filter algorithm for NaveGo INS/GPS system.
%
% INPUT:
%   x, 21x1 state vector.
%   z, 6x1 innovations vector.
%  dt, time period. 
%   S, data structure with the following fields:
%       F, 21x21 state transition matrix.
%       H,  6x21 observation matrix.
%       Q, 12x12 process noise covariance.
%       R,  6x6  observation noise covariance.
%       P, 21x21 a priori error covariance.
%       G, 21x12 control-input matrix.      
%
% INPUT:
%   xu, 21x1 a posteriori state vector.
%   xp, 21x1 a priori state vector.
%   S, the following fields are updated:
%       A,  21x21 state transition matrix.
%       K,  21x6  Kalman gain matrix.
%       Qd, 21x6  discrete process noise covariance.
%       P,  21x21 a posteriore error covariance.   
%       C,   6x6  innovation (or residual) covariance.
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
%			  R. Gonzalez, J. Giribet, and H. Patiño. NaveGo: a
% simulation framework for low-cost integrated navigation systems,
% Journal of Control Engineering and Applied Informatics, vol. 17,
% issue 2, pp. 110-120, 2015. Alg. 1.
%
% Version: 002
% Date:    2016/11/18
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

    else
        
        fprintf('* '); 
       
        i = find ( ti <= tg(j), 1, 'last');
    end

I = eye(max(size(S.F)));

% Step 1, update Kalman gain
S.C = (S.R + S.H * S.P * S.H');
S.K = (S.P * S.H') / (S.C) ;

% Step 2, update vector state
xu = x + S.K * (z - S.H * x);
% xu = S.K * z;                 % This expression can be used when x = 0.

% Step 3, update covariance matrix
Pu = (I - S.K * S.H) * S.P ;

% Discretization of continous-time system
S.A =  expm(S.F * dt);          % Exact expression
% S.A = I + (S.F * dt);         % Approximated expression
S.Qd = (S.G * S.Q * S.G') .* dt;

% Step 4, predict xp and Pp
xp = S.A * xu;
S.P = (S.A * Pu * S.A') + S.Qd;
S.P =  0.5 .* (S.P + S.P');

end
