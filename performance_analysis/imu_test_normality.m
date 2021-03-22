function pd = imu_test_normality(imu_sta, cdf)
% imu_test_normality: performs normality analysis on static inertial
% measurements.
%
% INPUT
%   imu_sta, input data structure must contains the following fields
%       fb: Nx3 accelerations [X Y Z] (m/s^2).
%       wb: Nx3 turn rates [X Y Z] (rad/s).
%       t: Nx1 time vector (s).
%
%   cdf, 'ON' or 'OFF' (string)
%
% OUTPUT
%    pd: probality distribution object from ProbabilityDistribution class.
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
% References:
%
%
% Version: 001
% Date:    2021/03/05
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

if nargin < 2, cdf = 'OFF', end

%% Test of normality for inertial sensors
%%

% Decimation or downsampling
DEC = 100;

fprintf('imu_test_normality: downsampling ratio is %d.\n', DEC );

imu_ds.fb = imu_sta.fb(1:DEC:end, :);
imu_ds.wb = imu_sta.wb(1:DEC:end, :);

for i=1:3
    
    data = imu_ds.fb(:,i);
    
    pd(i) = test_normality (data);
    
    x_label = sprintf('FB %d SAMPLES', i);
    x_title = sprintf('FB %d HISTOGRAM', i);
    plot_histogram (data, pd, x_label, x_title);
    
    if ( strcmp(cdf, 'ON') )
        x_label = sprintf('FB %d SAMPLES', i);
        x_title = sprintf('FB %d NORMAL CUMULATIVE DISTRIBUTION', i);
        plot_cdf (data, pd_imu_fb(i), x_label, x_title);
    end
end

for i=1:3
    
    data = imu_ds.wb(:,i);
    
    pd(i+3) = test_normality (data);
    
    x_label = sprintf('WB %d SAMPLES', i);
    x_title = sprintf('WB %d HISTOGRAM', i);
    plot_histogram (data, pd, x_label, x_title);
    
    if ( strcmp(cdf, 'ON') )
        x_label = sprintf('FB %d SAMPLES', i);
        x_title = sprintf('WB %d NORMAL CUMULATIVE DISTRIBUTION', i);
        plot_cdf (data, pd_imu_wb(i), x_label, x_title);
    end
end
