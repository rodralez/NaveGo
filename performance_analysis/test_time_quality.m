function [hk, pk, rmse_err ] = test_time_quality(time_v)
% test_time_quality: 
%
% INPUT
%   time_v: Nx1 time vector.
%
% OUTPUT
%   hk: = 0, samples come from a normal distribution.
%       = 1, samples do not come from a normal distribution.
%   pd: probality distribution object from ProbabilityDistribution class.
%   rmse_err: RMSE between the two curves.
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
%
% Version: 001
% Date:    2021/03/02
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

%% Statistics

time_d = diff(time_v);

dt1 = mean(time_d);
dt2 = median(time_d);
dt3 = mode(time_d);

fprintf('test_normality: time vector differences mean is   %.4f. \n', dt1)
fprintf('test_normality: time vector differences median is %.4f. \n', dt2)
fprintf('test_normality: time vector differences mode is   %.4f. \n', dt3)
fprintf('test_normality: time vector differences min is    %.4f. \n', min(time_d))
fprintf('test_normality: time vector differences max is    %.4f. \n', max(time_d))

dt = dt2;

% Histogram
bins = 10;
figure
histogram(time_d, bins, 'Normalization', 'count', 'FaceColor', [.9 .9 .9]); 
title('TIME DIFFERENCES HISTOGRAM')

%% Frequency

freq = 1 / dt;

fprintf('test_normality: time vector frequency is %.2f. \n', freq)

%% Test if there are repeated times

if (any(time_d))
    fprintf('test_normality: there are no repeated elements in time vector. \n')
else
    fprintf('test_normality: there are repeated elements in time vector. \n')
end

tu = unique(time_v);
[nu, mu] = size(tu);
[n, m] = size(time_v);

if (n == nu && m == mu)
    fprintf('test_normality: all elements in time vector are unique. \n')
else
    fprintf('test_normality: all elements in time vector are not unique. \n')
end

%% Check if size of time vector is coherence 

if n > m
    dim = n;
else
    dim = m;
end

% Hypotetical time
th = dt * dim; % sampling time times the number of elements 

fprintf('test_normality: vector time should be %.2f hours or %.2f minutes or %.2f seconds. \n', (th/60/60), (th/60), th)

to = (time_v(end) - time_v(1));

fprintf('test_normality: vector time is %.2f hours or %.2f minutes or %.2f seconds. \n', (to/60/60), (to/60), to)

%% Plots

orange_new = [0.8500 0.3250 0.0980];

time_d_s = sort(time_d);

figurefigure

for i = 2:n
    
    plot (time_v(i), time_d_s(i), 'o-', 'color', orange_new)
    hold on
    
end
plot (time_v(2:end), time_d_s, 'o-')
hold on
line ( [time_v(2), time_v(end)], [dt, dt] , 'color', orange_new, 'linewidth', 2, 'LineStyle','--')
title('SORTED TIME DIFFERENCES')
legend('Sorted time diff', 'Sampling time')
grid on

end
