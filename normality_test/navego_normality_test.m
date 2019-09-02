% navego_normality_test: example of how to use NaveGo functions for 
% test normality of a data vector
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
% Version: 002
% Date:    2019/09/02
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

clc
clear
close all
matlabrc

format SHORTE

addpath ./
addpath ./../

%% TEST A GAUSSIAN DISTRIBUTION

X = randn(10000,1);

[pd, ha] = normality_test (X);

if ~( ha )    
    disp('navego_normality_test: Data under analysis comes from a normal distribution.');
    
else
    disp('navego_normality_test: data under analysis does not come from a normal distribution.');
    
end

figure
plot_histogram (X, pd)
title ('HISTOGRAM')

figure
r = plot_cdf (X, pd);
title('NORMAL CUMULATIVE DISTRIBUTION')

fprintf('navego_normality_test: RMSE between ideal CDF and real CDF is %f \n', r)

%% TEST A UNIFORM DISTRIBUTION

Y = rand(10000,1);

[pd, ha] = normality_test (Y);

if ~( ha )    
    disp('navego_normality_test: Data under analysis comes from a normal distribution.');
    
else
    disp('navego_normality_test: data under analysis does not come from a normal distribution.');
    
end

figure
plot_histogram (Y, pd)
title ('HISTOGRAM')

figure
r = plot_cdf (X, pd);
title('NORMAL CUMULATIVE DISTRIBUTION')

fprintf('navego_normality_test: RMSE between ideal CDF and real CDF is %f \n', r)


