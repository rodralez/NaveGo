function [b_sta_n] = noise_b_sta (b_sta, M)
% noise_sbias: generates both a deterministic and a stochastic (run-to-run) 
% static bias error.
%
% INPUT
%		b_sta: 1x1 static bias to define the interval [-sbias sbias] 
%		M: dimension of output vector.
%
% OUTPUT
%		b_sta_n: Mx3 matrix with simulated static biases [X Y Z]
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
a = -b_sta * 0.1;
b =  b_sta * 0.1;

% Stochastic static biases are chosen randomly within the interval [-b_sta b_sta]
b_sta_stoc = a' + (b' - a') .* rand(3,1) ;

I = ones(M,1);

b_sta_n = [ b_sta_stoc(1).* I + b_sta(1), ... 
            b_sta_stoc(2).* I + b_sta(2), ... 
            b_sta_stoc(3).* I + b_sta(3) ];
