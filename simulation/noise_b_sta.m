function [sbias_n] = noise_b_sta (sta_bias, M)
% noise_sbias: generates both a deterministic and a stochastic (run-to-run) 
% static bias error.
%
% INPUT
%		sbias: 1x1 static bias to define the interval [-sbias sbias] (rad)
%		M: dimension of output vector.
%
% OUTPUT
%		sbias_n: Mx3 matrix with simulated static biases [X Y Z] (rad, rad, rad).
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
%	R. Gonzalez, J. Giribet, and H. Patiño. NaveGo: a
% simulation framework for low-cost integrated navigation systems,
% Journal of Control Engineering and Applied Informatics, vol. 17,
% issue 2, pp. 110-120, 2015. Sec. 2.2.
%
% Version: 002
% Date:    2020/11/26
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

% It is considered that the stochastic static bias is a 10% of the
% deterministic static bias
a = -sta_bias * 0.1;
b =  sta_bias * 0.1;

% Static biases are chosen randomly in the interval [-sbias sbias]
sta_bias_random = (b' - a') .* rand(3,1) + a';

I = ones(M,1);

sbias_n = [ sta_bias_random(1).* I + sta_bias(1), ... 
            sta_bias_random(2).* I + sta_bias(2), ... 
            sta_bias_random(3).* I + sta_bias(3)];
