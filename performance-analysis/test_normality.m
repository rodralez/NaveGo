function [pd, hk, pk] = test_normality (samples)
% test_normality: checks if samples come from a normal distribution.
%
% INPUT
%   samples: Nx1 samples to test.
%
% OUTPUT
%    pd: probality distribution object from ProbabilityDistribution class.
%    hk: = 0 => samples come from a normal distribution.
%        = 1 => samples do not come from a normal distribution.
%    pk: p-value.
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
% Date:    2020/11/19
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

%% Fit data to a normal disribution

pd = fitdist(samples, 'normal');

sig = pd.sigma;
mu = pd.mu;

ref = randn (1000,1) * sig + mu;

%% Test normality

% Kolmogorov–Smirnov test
[hk , pk] = kstest2 (samples, ref, 'alpha', 0.001);

% Others normality tests
% [hz , pz] = ztest  (samples, mu, sig)
% [ht , pt] = ttest  (samples, mu)
% [hc , pc] = chi2gof(samples)
% [hj , pj] = jbtest (samples)
% [hk , pk] = kstest (samples, 'CDF', pd, 'Alpha',0.5)
% [ha, pa, adstat, cv] = adtest (samples,'Distribution', pd)

end
