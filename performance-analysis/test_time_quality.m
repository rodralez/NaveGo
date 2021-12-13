function [idx_zero] = test_time_quality(time_v, verbose)
% test_time_quality:
%
% INPUT
%   time_v: Nx1 time vector.
%   verbose: 0, no output plotting.
%            1, output plotting.
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
% Version: 006
% Date:    2021/12/09
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

%%

if nargin < 2, verbose  = 1; end

%% STATISTICS

time_d = diff(time_v);      % time vector difference

dt1 = mean(time_d);
dt2 = median(time_d);
dt3 = mode(time_d);
sd = std(time_d);

dt = dt2;                   % sampling time is taken from median

fprintf('test_time_quality: time mean is %e. \n', dt1)
fprintf('test_time_quality: time differences median is %e. \n', dt2)
fprintf('test_time_quality: time differences mode is %e. \n', dt3)
fprintf('test_time_quality: time differences min is %e. \n', min(time_d))
fprintf('test_time_quality: time differences max is %e. \n', max(time_d))
fprintf('test_time_quality: time differences \x03C3 is %e. \n', sd)


%% FREQUENCY

freq = 1 / dt;

fprintf('test_time_quality: time vector frequency is %.2f. \n', freq)

%% TEST UNIQUE VALUES

% Negative diff
idx_neg = time_d < 0.0;
neg_n = sum(idx_neg);

% Diff equals to zero
idx_zero = time_d == 0.0;
zero_n = sum(idx_zero);

lp = time_d > 2 * dt2;      % Outliers greater than 2 times the median.
outliers = sum(lp);

fprintf('test_time_quality: time differences have %d negative numbers. \n', neg_n)
fprintf('test_time_quality: time differences have %d numbers equal to zero. \n', zero_n)
fprintf('test_time_quality: time differences have %d outliers (>2\x002Amedian). \n', outliers)

tu = unique(time_v);
[nu, mu] = size(tu);
[n, m] = size(time_v);

if (n == nu && m == mu)
    fprintf('test_time_quality: all elements in time are unique. \n')
else
    fprintf('test_time_quality: some elements in time  are not unique. \n')
end

if (all(time_d))
    fprintf('test_time_quality: all contiguous elements in time are different. \n')
else
    fprintf('test_time_quality: some contiguous elements in time are equal. \n')
end

%% CHECK TIMESPAN

if n > m
    dim = n;
else
    dim = m;
end

% Hypotetical time
th = dt * dim; % sampling time times the vector dimension

fprintf('test_time_quality: timespan should be %.2f hours or %.2f minutes or %.2f seconds. \n', (th/60/60), (th/60), th)

to = (time_v(end) - time_v(1));

fprintf('test_time_quality: timespan is %.2f hours or %.2f minutes or %.2f seconds. \n', (to/60/60), (to/60), to)

%% PLOTS

if (verbose == 1)
    
    blue    = [0, 0.4470, 0.7410];
    orange  = [0.8500, 0.3250, 0.0980];
    green   = [0.4660, 0.6740, 0.1880];
    yellow  = [0.9290, 0.6940, 0.1250];
    light_blue = [0.3010, 0.7450, 0.9330];
    
    % Text size
    font_title  = 50;
    font_tick   = 15;
    font_label  = 20;
    font_legend = 15;
    
    % Line width
    lw = 3;

    % Histogram
    figure
    if (is_octave)
      hist(time_d, 'FaceColor', [.9 .9 .9]);
    else
      histogram(time_d, 'BinMethod', 'sturges', 'Normalization', 'count', 'FaceColor', [.9 .9 .9]);
    end


    t1 = title('TIME DIFFERENCE HISTOGRAM');
    x1 = xlabel('\Delta time [s]'); 
    y1 = ylabel('Samples');
    
    set(t1,'FontSize', font_title);
    set(x1,'FontSize', font_label);
    set(y1,'FontSize', font_label);
    set(gca, 'YTickMode', 'auto', 'FontSize', font_tick);
    
    % Time differences
    figure
    
    plot (time_v(~idx_zero), time_d(~idx_zero), 'x', 'LineWidth', lw);
    hold on
    plot (time_v(idx_zero), time_d(idx_zero), '+r', 'LineWidth', lw);
    line ( [time_v(2), time_v(end)], [dt, dt] , 'color', orange, 'linewidth', lw, 'LineStyle','--')
    x1 = xlabel('Time [s]');
    y1 = ylabel('\Delta time [s]'); 
    t1 = title('TIME DIFFERENCES');
    l1 = legend('Time differences', 'Median sampling time');
    grid on
    
    set(t1,'FontSize', font_title);
    set(x1,'FontSize', font_label);
    set(y1,'FontSize', font_label);   
    set(l1,'FontSize', font_legend);
    set(gca, 'YTickMode', 'auto', 'FontSize', font_tick);
    
    % Sorted time differences
    time_d_s = sort(time_d);
    M = max(size(time_d_s));
    
    figure
    
    plot (1:M, time_d_s, 'x', 'LineWidth', lw);
    hold on
    line ( [1 M], [dt, dt] , 'color', orange, 'linewidth', lw, 'LineStyle','--')
    x1 = xlabel('Samples');
    y1 = ylabel('\Delta time [s]');
    t1 = title('SORTED TIME DIFFERENCES');
    l1 = legend('Sorted time diff', 'Median sampling time');
    grid on
    
    set(t1,'FontSize', font_title);
    set(x1,'FontSize', font_label);
    set(y1,'FontSize', font_label);    
    set(l1,'FontSize', font_legend);
    set(gca, 'YTickMode', 'auto', 'FontSize', font_tick);
    
    % Time regularity
    figure
    
    plot (time_v, 'x-', 'LineWidth', lw);
    t1 = title('TIME REGULARITY');
    x1 = xlabel('Samples');
    y1 = ylabel('Time [s]');
    grid on
    
    set(t1,'FontSize', font_title);
    set(x1,'FontSize', font_label);
    set(y1,'FontSize', font_label);
    set(gca, 'YTickMode', 'auto', 'FontSize', font_tick);
    
end
end
