function [rrw_n] = noise_rrw (rrw, dt, M)
% noise_rrw: generates rate random walk noise.
%
% INPUT
%		rrw: 1x3 level of rate random walk.
%		dt: 1x1 sampling time.
%   M: 1x2 dimensionx of output vector.
%
% OUTPUT
%		rrw_n: M matrix with simulated rate random walk noise [X Y Z] 
%     (rad/s^2, rad/s^2, rad/s^2).
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
%
% Version: 001
% Date:    2017/07/28
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

rrw_n = zeros(M);
N = M(1);

for i=1:3
    
    b_noise = randn(N-1,1);
    
    for j=2:N
        rrw_n (j, i) = rrw_n(j-1, i) + rrw(i) * dt .* b_noise(j-1);
    end
end
