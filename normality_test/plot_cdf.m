function rmse_err = plot_cdf (samples, pd)
% plot_cdf: plots cumulative distribution function (CDF) from samples 
% (empirical CDF) and compares to inferred CDF (reference CDF)
%
% INPUT
%   samples: Nx1 samples.
%   pd: probality distribution object from ProbabilityDistribution class.
%
% OUTPUT
%   rmse_err: RMSE between the two curves.
%   figure with empirical CDF and reference CDF.
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
% Date:    2019/09/02
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

%% PLOT PARAMETERS

font_tick = 12;
font_label = 16;
line_wd = 3;

blue_new = [0 0.4470 0.7410];
orange_new = [0.8500 0.3250 0.0980];

%% Only plot 1-sigma elements

M = 1;

sig = pd.sigma;
mu = pd.mu;

edge = abs(M * sig + mu);
idx = find (samples > -edge, 1, 'first');
fdx = find (samples <  edge, 1, 'last');
samples = samples(idx:fdx);

%% IDEAL CDF

N = length(samples);
x = linspace(min(samples), max(samples), N );
ideal_cdf = normcdf(x, mu, sig)';

%% REAL CDF

x_sort = sort(samples);
real_cdf = ( (1:N) - 0.5)' ./ N;

% Root mean squared error
rmse_err = rmse(ideal_cdf, real_cdf);

%% PLOT

h = plot(x, ideal_cdf, '--',  'LineWidth', 2, 'Color', orange_new);
hold on

s = stairs(x_sort, real_cdf,'-.', 'LineWidth', 2, 'Color', blue_new);

legend([h, s], 'Ideal CDF', 'Real CDF' )

xl = xlabel('Samples');
% yl = ylabel('Cumulative probability (CDF)');

grid
hold off

set(h,'Linewidth', line_wd );
set(gca, 'YTickMode', 'auto', 'FontSize', font_tick);

set(xl,'FontSize', font_label);
% set(yl,'FontSize', font_label);

end