function plot_histogram (samples, pd, x_label, x_title)
% plot_histogram: plots histogram from samples (empirical PDF)  and 
% compares to inferred probability density function (reference PDF). It 
% also plots mean and median.
%
% INPUT
%   samples: Nx1 samples.
%   pd: probality distribution object from ProbabilityDistribution class.
%   x_label: label for X axis (string).
%   x_title: title for the figure (string).
%
% OUTPUT
%   figure with histogram, reference PDF, mean and median.
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
% Version: 004
% Date:    2021/03/02
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego`

N = length(samples);

%% REFERENCE PDF

x = linspace(min(samples), max(samples), N );
ref_pdf = pdf(pd, x);

%% STATISTIC ANALYSIS

mu = mean(samples);
med = median(samples);

i = -5;
idx1 = [];
idx2 = [];

while (isempty(idx1) || isempty(idx2))
    
    EPS = 10^i;
    idx1 = find( x >= mu - EPS   & x < mu + EPS );
    idx2 = find( x >= med - EPS & x < med + EPS );
    i = i + 1;    
end

if ( isempty(idx1) || isempty(idx2) )
    error('plot_histogram: no match for mean or median')
end

% Middlepoints 
idx1 = idx1( ceil(end/2) );
idx2 = idx2( ceil(end/2) );

%% PLOT

bins = 100;
blue_new = [0 0.4470 0.7410];
orange_new = [0.8500 0.3250 0.0980];

figure

% Plot histogram from dataquiq
histogram(samples, bins, 'Normalization', 'pdf', 'FaceColor', [.9 .9 .9]);
hold on

% Plot the reference pdf
p0 = plot(x, ref_pdf, '-',  'LineWidth', 2);

% Plot lines
y = ref_pdf (idx1);
l1 = line( [mu, mu] , [0, y], 'Color', blue_new, 'LineWidth', 2, 'LineStyle','-.');

y = ref_pdf (idx2);
l2 = line( [med, med] , [0, y], 'Color', orange_new, 'LineWidth', 2, 'LineStyle','-.' );

legend([p0, l1, l2], 'Reference PDF', 'Mean', 'Median')

xlabel(x_label);
% ylabel('Probability density function (PDF)');
title(x_title)

grid on
hold off

end
