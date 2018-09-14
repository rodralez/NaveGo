function [dbias_n] = noise_dbias (b_corr, b_drift, dt, M)
% noise_dbias: generates a dynamic bias perturbation.
%
% INPUT:
%		b_corr: 1x3, correlation times.
%       b_drift: 1x3, level of dynamic biases.
%       dt: sample time.
%		M: 1x2, dimension of output vector.
%
% OUTPUT:
%		dbias_n: M matrix with [x, y, z] simulated dynamic biases.
%
%   Copyright (C) 2014, Rodrigo González, all rights reserved.
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
%           Aggarwal, P. et al. MEMS-Based Integrated Navigation. Artech
% House. 2010. Eq. 3.33, page 57.
%
% Version: 001
% Date:    2017/07/27
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

% If correlation time is provided...
if (~isinf(b_corr))

    % Gauss-Markov process
    dbias_n = zeros(M);
    N = M(1);

    for i=1:3

        beta  = dt / ( b_corr(i) );
        sigma = b_drift(i);
        a1 = exp(-beta);
        a2 = sigma * sqrt(1 - exp(-2*beta) );

        b_noise = randn(N-1,1); 
        
        for j=2:N
            dbias_n(j, i) = a1 * dbias_n(j-1, i) + a2 .* b_noise(j-1);
        end
    end
    
% If not...
else
    sigma = b_drift;
    bn = randn(M);
    
    dbias_n = [sigma(1) .* bn(:,1), sigma(2) .* bn(:,2), sigma(3) .* bn(:,3)];
    
end