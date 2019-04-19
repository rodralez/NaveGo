function  kf = kalman(kf, dt)
% kalman: Kalman filter algorithm.
%
% INPUT
%   kf: data structure with at least the following fields:
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
% OUTPUT
%    kf: the following fields are updated:
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
%   R. Gonzalez, J. Giribet, and H. Patiño. NaveGo: a
% simulation framework for low-cost integrated navigation systems,
% Journal of Control Engineering and Applied Informatics, vol. 17,
% issue 2, pp. 110-120, 2015. Alg. 1.
%
%   Dan Simon. Optimal State Estimation. Chapter 5. John Wiley 
% & Sons. 2006.   
%
% Version: 007
% Date:    2019/04/19
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

% PREDICTION STEP

kf = kf_prediction(kf, dt);

% UPDATE STEP

kf = kf_update(kf);

end
