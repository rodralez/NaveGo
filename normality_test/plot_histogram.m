function plot_histogram (samples, pd)
% plot_histogram: plots histogram from samples (empirical PDF)  and
% compares to inferred probability density function (reference PDF). It
% also plots mean and median.
%
% INPUT
%   samples: Nx1 samples.
%   pd: probality distribution object from ProbabilityDistribution class.
%
% OUTPUT
%   figure with histogram, ideal PDF, mean and median.
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
% Version: 002
% Date:    2019/09/02
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

%% PLOT PARAMETERS

font_tick = 12;
font_label = 16;
line_wd = 3;

bins = 100;
blue_new = [0 0.4470 0.7410];
orange_new = [0.8500 0.3250 0.0980];

%% Only plot 1-sigma elements

M = 1;

sig = pd.sigma;
mu = pd.mu;
med = median(samples);

edge = abs(M * sig + mu);
idx = find (samples > -edge, 1, 'first');
fdx = find (samples <  edge, 1, 'last');
samples = samples(idx:fdx);

%% IDEAL PDF

N = length(samples);

x = linspace( min(samples), max(samples), N );
ideal_pdf = pdf(pd, x);

%% STATISTIC ANALYSIS

EPS = 1E-3;

idx1 = find( x >= mu - EPS   & x < mu + EPS );
idx2 = find( x >= med - EPS & x < med + EPS );

if ( isempty(idx1) || isempty(idx2) )
    error('plot_histogram: no match for mean or median')
end

% Middlepoints
idx1 = idx1( ceil(end/2) );
idx2 = idx2( ceil(end/2) );

%% PLOT

% Plot histogram from data
histogram(samples, bins, 'Normalization', 'pdf', 'FaceColor', [.9 .9 .9]);
hold on

% Plot the ideal pdf
h = plot(x, ideal_pdf, '-',  'LineWidth', 2);

% Plot mean
y = ideal_pdf (idx1);
l1 = line( [mu, mu] , [0, y], 'Color', blue_new, 'LineWidth', line_wd, 'LineStyle','-.');

% Plot median
y = ideal_pdf (idx2);
l2 = line( [med, med] , [0, y], 'Color', orange_new, 'LineWidth', line_wd, 'LineStyle', '--' );

legend([h, l1, l2], 'Ideal PDF', 'Mean', 'Median')

xl = xlabel('Samples');
% yl = ylabel('Probability density function (PDF)');

grid on
hold off

set(h,'Linewidth', line_wd );
set(gca, 'YTickMode', 'auto', 'FontSize', font_tick);

set(xl,'FontSize', font_label);
% set(yl,'FontSize', font_label);

end