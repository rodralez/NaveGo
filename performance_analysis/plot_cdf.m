function rmse_err = plot_cdf (data, pd, x_label, x_title)
% plot_cdf: plots cumulative distribution function (CDF) from samples 
% (empirical CDF) and compares to inferred CDF (reference CDF)
%
% INPUT
%   samples: Nx1 samples.
%   pd: probality distribution object from ProbabilityDistribution class.
%   x_label: label for X axis (string).
%   x_title: title for the figure (string).
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
% Date:    2021/03/02
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

%% REFERENCE CDF

N = length(data);
sig = pd.sigma;
mu = pd.mu;
x = linspace(min(data), max(data), N );
ref_cdf = normcdf(x, mu, sig)';

%% EMPIRICAL CDF

x_sort = sort(data);
emp_cdf = ( (1:N) - 0.5)' ./ N;

% Root mean squared error
rmse_err = rmse(ref_cdf, emp_cdf);

%% PLOT

blue_new = [0 0.4470 0.7410];
orange_new = [0.8500 0.3250 0.0980];

figure 

p1 = plot(x, ref_cdf, '-.',  'LineWidth', 2, 'Color', orange_new);
hold on
p2 = stairs(x_sort, emp_cdf,'-', 'LineWidth', 2, 'Color', blue_new);

xlabel(x_label);
ylabel('Cumulative probability (CDF)');
legend([p1, p2], 'Reference CDF', 'Empirical CDF' )
title(x_title)

grid
hold off

end

