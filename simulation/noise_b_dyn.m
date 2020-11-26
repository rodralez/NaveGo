function [b_dyn_n] = noise_b_dyn (b_corr, b_dyn, dt, M)
% noise_dbias: generates a dynamic bias perturbation.
%
% INPUT
%	b_corr: 1x3 correlation times.
%   b_dyn: 1x3 level of dynamic biases.
%   dt: 1x1 sampling time.
%	M: 1x2 dimension of output vector.
%
% OUTPUT
%	b_dyb_n: M matrix with simulated dynamic biases [X Y Z] 
%     (rad/s, rad/s, rad/s).
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
%
%   Aggarwal, P. et al. MEMS-Based Integrated Navigation. Artech
% House. 2010. Eq. 3.33, page 57.
%
% Version: 003
% Date:    2020/11/26
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

% If correlation time is provided...
if (~isinf(b_corr))

    % First-order Gauss-Markov process
    b_dyn_n = zeros(M);
    N = M(1);           % Number of rows

    for i=1:3

        beta  = dt / ( b_corr(i) );
        sigma = b_dyn(i);
        a1 = exp(-beta);
        
        % The dynamic bias noise variance is modeled as an exponentially correlated
        % fixed-variance first-order Markov process
        sigma_gm = sigma * sqrt(1 - exp(-2*beta) );

        b_wn = sigma_gm .* randn(N,1); 
        
        for j=2:N
            b_dyn_n(j, i) = a1 * b_dyn_n(j-1, i) +  b_wn(j-1);
        end
    end
    
% If not...
else
    sigma = b_dyn;
    b_wn = randn(M);
    
    b_dyn_n = [sigma(1).*b_wn(:,1) , sigma(2).*b_wn(:,2) , sigma(3).*b_wn(:,3)];    
end
