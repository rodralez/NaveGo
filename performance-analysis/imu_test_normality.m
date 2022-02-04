function pd = imu_test_normality(imu_sta, dec, cdf)
% imu_test_normality: performs normality analysis on static inertial
% measurements.
%
% INPUT
%   imu_sta, input data structure must contains the following fields
%       fb: Nx3 accelerations [X Y Z] (m/s^2).
%       wb: Nx3 turn rates [X Y Z] (rad/s).
%       t: Nx1 time vector (s).
%
%   dec, decimation factor (integer)
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
% Version: 003
% Date:    2021/12/02
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

if nargin < 2, dec = 100; end

if nargin < 3, cdf = 'OFF'; end

fprintf('imu_test_normality: CDF test is %s.\n', cdf );

%% Test of normality for inertial sensors
%%

dec = floor(dec);       % Force dec to be integer

fprintf('imu_test_normality: downsampling ratio is %d.\n', dec );

imu_ds.fb = imu_sta.fb(1:dec:end, :);
imu_ds.wb = imu_sta.wb(1:dec:end, :);

for i=1:3
    
    data = imu_ds.fb(:,i);
    
    pd(i) = test_normality (data);
    
    x_label = sprintf('ACCR %d SAMPLES', i);
    x_title = sprintf('ACCR %d HISTOGRAM', i);
    plot_histogram (data, pd(i), x_label, x_title);
    
    if ( strcmp(cdf, 'ON') )
        x_label = sprintf('ACCR %d SAMPLES', i);
        x_title = sprintf('ACCR %d NORMAL CUMULATIVE DISTRIBUTION', i);
        plot_cdf (data, pd(i), x_label, x_title);
    end
end

for i=1:3
    
    data = imu_ds.wb(:,i);
    
    pd(i+3) = test_normality (data);
    
    x_label = sprintf('GYRO %d SAMPLES', i);
    x_title = sprintf('GYRO %d HISTOGRAM', i);
    plot_histogram (data, pd(i+3), x_label, x_title);
    
    if ( strcmp(cdf, 'ON') )
        x_label = sprintf('GYRO %d SAMPLES', i);
        x_title = sprintf('GYRO %d NORMAL CUMULATIVE DISTRIBUTION', i);
        plot_cdf (data, pd(i+3), x_label, x_title);
    end
end
