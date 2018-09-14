function [sbias_n] = noise_sbias (sbias, N)
% noise_sbias: generates a random static bias perturbation.
%
% INPUT:
%		sbias: static bias from [-sbias sbias].
%		N: dimension of output vector.
%
% OUTPUT:
%		sbias_n: Nx3 matrix with [x, y, z] simulated static biases.
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
%			R. Gonzalez, J. Giribet, and H. Patiño. NaveGo: a
% simulation framework for low-cost integrated navigation systems,
% Journal of Control Engineering and Applied Informatics, vol. 17,
% issue 2, pp. 110-120, 2015. Sec. 2.2.
%
% Version: 001
% Date:    2017/07/27
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

a = -sbias;
b =  sbias;
ab_fix = (b' - a') .* rand(3,1) + a';
o = ones(N,1);
sbias_n = [ab_fix(1).* o   ab_fix(2).* o   ab_fix(3).* o];