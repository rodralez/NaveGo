function [idx_zero] = test_time_quality(time_v)
% test_time_quality: 
%
% INPUT
%   time_v: Nx1 time vector.
%
% OUTPUT
%   idx_zero: (N-1)x1 logical index where time difference is equal to zero
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
% Version: 003
% Date:    2021/11/29
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

%% Statistics

time_d = diff(time_v);  % time vector difference

dt1 = mean(time_d);
dt2 = median(time_d);
dt3 = mode(time_d);
sd = std(time_d);

dt = dt2;                   % sampling time is taken from median

% Negative diff
idx_neg = time_d < 0.0;
neg_n = sum(idx_neg);

% Diff equals to zero
idx_zero = time_d == 0.0;
zero_n = sum(idx_zero);

lp = time_d > 2 * dt2;     % Outliers greater than 2 times the median.
pos_numbers = sum(lp);
 
fprintf('test_time_quality: time vector difference mean is %e. \n', dt1)
fprintf('test_time_quality: time vector difference median is %e. \n', dt2)
fprintf('test_time_quality: time vector difference mode is %e. \n', dt3)
fprintf('test_time_quality: time vector difference min is %e. \n', min(time_d))
fprintf('test_time_quality: time vector difference max is %e. \n', max(time_d))
fprintf('test_time_quality: time vector difference \x03C3 is %e. \n', sd)
fprintf('test_time_quality: time vector difference has %d negative numbers. \n', neg_n)
fprintf('test_time_quality: time vector difference has %d numbers equal to zero. \n', zero_n)
fprintf('test_time_quality: time vector difference has %d numbers greater than 2 \x002A median. \n', pos_numbers)

% Histogram
figure
histogram(time_d, 'BinMethod', 'sturges', 'Normalization', 'count', 'FaceColor', [.9 .9 .9]); 
title('TIME DIFFERENCE HISTOGRAM')

%% Frequency

freq = 1 / dt;

fprintf('test_time_quality: time vector frequency is %.2f. \n', freq)

%% Test if there are repeated times

if (all(time_d))
    fprintf('test_time_quality: all contiguous elements in time vector are different. \n')
else
    fprintf('test_time_quality: some contiguous elements in time vector are equal. \n')
end

tu = unique(time_v);
[nu, mu] = size(tu);
[n, m] = size(time_v);

if (n == nu && m == mu)
    fprintf('test_time_quality: all elements in time vector are unique. \n')
else
    fprintf('test_time_quality: some elements in time vector are not unique. \n')
end

%% Check if size of time vector is coherence 

if n > m
    dim = n;
else
    dim = m;
end

% Hypotetical time
th = dt * dim; % sampling time times the number of elementest_normalityts 

fprintf('test_time_quality: vector time should be %.2f hours or %.2f minutes or %.2f seconds. \n', (th/60/60), (th/60), th)

to = (time_v(end) - time_v(1));

fprintf('test_time_quality: vector time is %.2f hours or %.2f minutes or %.2f seconds. \n', (to/60/60), (to/60), to)

%% PLOTS

orange_new = [0.8500 0.3250 0.0980];

figure

plot (time_v(~idx_zero), time_d(~idx_zero), 'xb')
hold on
plot (time_v(idx_zero), time_d(idx_zero), '+r')
line ( [time_v(2), time_v(end)], [dt, dt] , 'color', orange_new, 'linewidth', 2, 'LineStyle','--')
title('TIME DIFFERENCES')
legend('time diff', 'Sampling time')
grid on


time_d_s = sort(time_d);

figure

plot (time_v(2:end), time_d_s, 'o-')
hold on
line ( [time_v(2), time_v(end)], [dt, dt] , 'color', orange_new, 'linewidth', 2, 'LineStyle','--')
title('SORTED TIME DIFFERENCES')
legend('Sorted time diff', 'Sampling time')
grid on


figure

plot (time_v, 'x-')
title('TIME REGULARITY')
grid on

end
