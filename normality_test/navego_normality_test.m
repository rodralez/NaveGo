% navego_normality_test: script to evaluate functions related to test normality
% from a samples vector
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
% Version: 001
% Date:    2019/04/03
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

clc
clear
close all
matlabrc

format SHORTE

addpath ./

X = randn(10000,1);

pd = normality_test (X);

figure
plot_histogram (X, pd)
title ('HISTOGRAM')

figure
r = plot_cdf (X, pd);
title('NORMAL CUMULATIVE DISTRIBUTION')

fprintf('RMSE between expected CDF and real CDF is %f \n', r)


