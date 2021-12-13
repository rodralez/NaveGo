function plot_imu_sta(omega, freq, text_st)
% plot_imu_sta: plots static IMU data.
%
% INPUT
%   omega, IMU sensor data (rad/s or m/s^2).%   
%   freq, Nx1 IMU sensor sampling frequency (Hz).
%   text_st, title for the figure (string).
%
% OUTPUT
%   a figure.
%
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
%   M.A. Hopcroft. Allan overlap MATLAB function v2.24.
% https://www.mathworks.com/matlabcentral/fileexchange/13246-allan 
% 
% Version: 001
% Date:    2021/12/06
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

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

M = max(size(omega)) - 1;
dt = 1/freq;

time = (0:dt:dt*M)';

s.linear = polyfit( time(1:length(omega)), omega, 1);

% scale to median for plotting
omega_median=median(omega);
medianfreq=omega-omega_median;

% Screen for outliers using 5x Median Absolute Deviation (MAD) criteria
MAD = median(abs(medianfreq)/0.6745);

% adjust time to remove any starting offset
dtime = time - time(1) + mean(diff(time));

% plot the frequency data, centered on median
% this should not be necessary, but dsplot 1.1 is a little bit brittle
if size(dtime,2) > size(dtime,1), dtime=dtime'; end

% dsplot makes a new figure
if(is_octave)
  hd=plot(dtime,medianfreq);
else
  hd=dsplot(dtime,medianfreq);
end

  set(hd,'Marker','.','LineStyle','none'); % equivalent to '.-'
hold on;

fx = xlim;
% plot([fx(1) fx(2)],[omega_median omega_median],'-k');
plot([fx(1) fx(2)],[0 0],':k','LineWidth', lw);

% show 5x Median Absolute deviation (MAD) values
hm = plot([fx(1) fx(2)],[5*MAD 5*MAD],'-r','LineWidth', lw);
plot([fx(1) fx(2)],[-5*MAD -5*MAD],'-r','LineWidth', lw);


% show linear fit line
hf=plot(xlim,polyval(s.linear,xlim)-omega_median,'-g','LineWidth', lw);
t1 = title(text_st);

hs=plot(xlim,polyval(s.linear,xlim)-omega_median-3*MAD,'--m','LineWidth', lw);
plot(xlim,polyval(s.linear,xlim)-omega_median+3*MAD,'--m','LineWidth', lw);

%set(get(gca,'Title'),'Interpreter','none');
x1=xlabel('Time [sec]');
y1=ylabel('Samples');

if MAD ~= 0.0
    l1=legend([hd hm hs hf],{'data (centered on median)','5x MAD outliers', ...
        '3x MAD outliers', ['Linear Fit (' num2str(s.linear(1),'%g') ')']});
else
    l1=legend([hd hf],{'data (centered on median)', ...
        ['Linear Fit (' num2str(s.linear(1),'%g') ')']});
end

% tighten up
xlim([dtime(1) dtime(end)]);

set(t1,'FontSize', font_title);
set(x1,'FontSize', font_label);
set(y1,'FontSize', font_label);
set(l1,'FontSize', font_legend);
set(gca, 'YTickMode', 'auto', 'FontSize', font_tick);
