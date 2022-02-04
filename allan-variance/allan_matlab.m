function [N, K, B, tauB] = allan_matlab (data, Fs, text_st)
% allan_matlab: performs overlapping Allan variance analysis on a inertial  
% sensor measurements.
%
% INPUT
%   data, Nx1 data from inertial sensor
%   Fs, 1x1 sampling frequency
%
% OUTPUT
%   N, random walk coefficient.
%   vrw, 1x3 velocity random walk (m/s^2/root(Hz)). 
%   arw, 1x3 angle random walk (rad/s^2/root(Hz)). 
%       for accelerometers. Value is taken from the plot at t = 1 s.       
%       Note: m/s^2/root(Hz) = m/(s^2*(1/s)^(1/2)) = m*(1/s)^(3/2) =
%           = m/s/root(s).
%       Note: rad/s/root(Hz) = rad/(s*(1/s)^(1/2)) = rad*(1/s)^(1/2) =
%           = rad/root(s
%
%   K, rate random walk coefficient.
%   vrrw, 1x3 velocity rate random walk (m/s^3/root(Hz)). 
%   arrw, 1x3 angle rate random walk (rad/s^2/root(Hz)). 
%       
%   B, bias instability coefficient.
%   ab_dyn, 1x3 accrs bias instability (m/s^2).
%   gb_dyn: 1x3 gyros bias instability (rad/s). 
%
%   tauB, bias correlation time (s).
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
%   IEEE-SA Standards Board. IEEE Standard Specification Format
% Guide and Test Procedure for Single-Axis Interferometric Fiber Optic
% Gyros. ISBN 1-55937-961-8. September 1997.
%
%   Naser El-Sheimy et at. Analysis and Modeling of Inertial Sensors
% Using Allan Variance. IEEE TRANSACTIONS ON INSTRUMENTATION AND
% MEASUREMENT, VOL. 57, NO. 1, JANUARY 2008.
%
%   Oliver J. Woodman. An introduction to inertial navigation. Technical
% Report. ISSN 1476-2986. University of Cambridge, Computer Laboratory.
% August 2007.
%
%   Most of this code has been taken from 
%   https://www.mathworks.com/help/nav/ug/inertial-sensor-noise-analysis-using-allan-variance.html
%
% Version: 001
% Date:    2021/12/02
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

%% Overlapping Allan variance
%%

t0 = 1/Fs;

theta = cumsum(data, 1)*t0;

maxNumM = 100;
L = size(theta, 1);
maxM = 2.^floor(log2(L/2));
m = logspace(log10(1), log10(maxM), maxNumM).';
m = ceil(m); % m must be an integer.
m = unique(m); % Remove duplicates.

tau = m*t0;

avar = zeros(numel(m), 1);
for i = 1:numel(m)
    mi = m(i);
    avar(i,:) = sum( ...
        (theta(1+2*mi:L) - 2*theta(1+mi:L-mi) + theta(1:L-2*mi)).^2, 1);
end
avar = avar ./ (2*tau.^2 .* (L - 2*m));

adev = sqrt(avar);

% figure
% loglog(tau, adev)
% title('Allan Deviation')
% xlabel('\tau');
% ylabel('\sigma(\tau)')
% grid on
% axis equal

%% Random Walk
%%

% Find the index where the slope of the log-scaled Allan deviation is equal
% to the slope specified.
slope = -0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);% Find the index where the slope of the log-scaled Allan deviation is equal

% Determine the angle random walk coefficient from the line.
logN = slope*log(1) + b;
N = 10^logN;

% Plot the results.
tauN = 1;
lineN = N ./ sqrt(tau);

% figure
% loglog(tau, adev, tau, lineN, '--', tauN, N, 'o')
% title('Allan Deviation with Angle Random Walk')
% xlabel('\tau')
% ylabel('\sigma(\tau)')
% legend('\sigma', '\sigma_N')
% text(tauN, N, 'N')
% grid on
% axis equal

%% Rate Random Walk
%%

% Find the index where the slope of the log-scaled Allan deviation is equal
% to the slope specified.
slope = 0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the rate random walk coefficient from the line.
logK = slope*log10(3) + b;
K = 10^logK;

% Plot the results.
tauK = 3;
lineK = K .* sqrt(tau/3);

% figure
% loglog(tau, adev, tau, lineK, '--', tauK, K, 'o')
% title('Allan Deviation with Rate Random Walk')
% xlabel('\tau')
% ylabel('\sigma(\tau)')
% legend('\sigma', '\sigma_K')
% text(tauK, K, 'K')
% grid on
% axis equal

%% Bias Instability
%%
% Find the index where the slope of the log-scaled Allan deviation is equal
% to the slope specified.
slope = 0;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the bias instability coefficient from the line.
scfB = sqrt(2*log(2)/pi);
logB = b - log10(scfB);
B = 10^logB;

% Plot the results.
tauB = tau(i);
lineB = B * scfB * ones(size(tau));% Static bias 
imu.ab_sta = zeros(1,3);
imu.gb_sta = zeros(1,3);

% figure
% loglog(tau, adev, tau, lineB, '--', tauB, scfB*B, 'o')
% title('Allan Deviation with Bias Instability')
% xlabel('\tau')
% ylabel('\sigma(\tau)')
% legend('\sigma', '\sigma_B')
% text(tauB, scfB*B, '0.664B')
% grid on
% axis equal

%% All plots togheter
%%

% Colors
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

tauParams = [tauN, tauK, tauB, tauB];
params = [N, K, scfB*B, 1e-5];

figure
loglog(tau, adev, tau, [lineN, lineK, lineB], '--', ...
    tauParams, params, 'o', 'LineWidth', lw)

my_title = sprintf('ALLAN VARIANCE FOR %s', cell2mat(text_st));

t1 = title(my_title);
x1 = xlabel('\tau');
y1 = ylabel('\sigma(\tau)');
l1 = legend('Allan variance', '-1/2 slope', ' 1/2 slope', ' 0 slope'); % , 'Interpreter', 'latex'

tt = text(tauParams, params*1.25, {['N=',num2str(N,'%.3E')] , ['K=',num2str(K,'%.3E')], ...
                              ['B=',num2str(B,'%.3E')], ['tauB=',num2str(tauB,'%.4f')] } );
grid on
axis tight
% ylim([1e-5 1e-2])

set(t1,'FontSize', font_title);
set(x1,'FontSize', font_label);
set(y1,'FontSize', font_label);
set(tt,'FontSize', font_tick);
set(l1,'FontSize', font_legend);
set(gca, 'YTickMode', 'auto', 'FontSize', font_tick);
