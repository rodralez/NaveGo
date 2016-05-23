function  [xu, Pp, K, xp] = kalman(x, y, F, H, G, P, Q, R, dt)
% kalman: Kalman filter algorithm
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
% Version: 001
% Date:    2014/09/11
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

I = eye(max(size(F)));

% Step 1, update Kalman gain
IS = (R + H * P * H');
K = (P * H') / (IS) ;

% Step 2, update vector state
% xu = x + K * (y - H*x);
xu = K*y;

% Step 3, update covariance matrix
Pu = (I - K*H) * P ;

% *************************************************************************
% Sequential KF
% *************************************************************************
% l = max(size(R));
% n = max(size(x));
% xu = zeros(n,1);
% K = zeros(n,l);
% I = eye(n);
% Pu = P;
%
% for j = 1:l
%
%     h = H(j,:);
%     r = R(j,j);
%     alpha = h * Pu * h' + r;
%     k = Pu * h' / alpha;
%     xu = xu + k*y(j);
%     K(:,j) = k;
%     Pu = (I - k*h) * Pu ;
% end

% *************************************************************************

% Discretization of the continous-time system.
% A =  expm(F*dt);
A = I + (F*dt);
Qd = (G * Q * G') .* dt;

% Step 4, predict Pp
xp = A * xu;
Pp = (A * Pu * A') + Qd;
Pp =  0.5 .* (Pp + Pp');

end
