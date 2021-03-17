function  navego_plot (cell_s, TYPE, ref_t, ref_var, nav_t, nav_var, gnss_t, gnss_var)
% navego_plot: plots results from INS/GNSS integration dataset.
%
% INPUT:
%   cell_s,  cell with strings (cell).
%   TYPE,    'NORMAL' or 'ERROR' (string).
%   ref_t,   Rx1 reference time (s)
%   ref_var, Rx1reference variable
%   nav_t,   Nx1 INS/GNSS time vector.
%   nav_var, Nx1 INS/GNSS variable.
%   gnss_t,  Gx1 GNSS time vector.
%   gnss_var, Gx1 GNSS time variable.
%
% OUTPUT
%   One figure.
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
% Version: 001
% Date:    2021/03/15
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

%% PARAMETERS
%%

% Colors
blue    = [0, 0.4470, 0.7410];
orange  = [0.8500, 0.3250, 0.0980];
% green   = [0.4660, 0.6740, 0.1880];
% yellow  = [0.9290, 0.6940, 0.1250];
% light_blue = [0.3010, 0.7450, 0.9330];

% Text size
% font_title  = 50;
% font_label  = 25;
% font_tick   = 15;
% font_legend = 15;
font_title  = 25;
font_label  = 15;
font_tick   = 10;
font_legend = 10;

% Line width
lw = 2;

%% PLOT
%%

p1 = plot(ref_t, ref_var, '--k');

hold on

if (strcmp(TYPE,'ERROR'))
    plot(ref_t, -ref_var, '--k');
end

% If GNSS data is available...
if nargin > 7
    
    p2 = plot(gnss_t, gnss_var, '-.', 'Color', orange, 'Linewidth', lw);
end

p3 = plot(nav_t, nav_var, '-.', 'Color', blue, 'Linewidth', lw);

hold off
grid on

%% Legend rules
%%

% If GNSS data is not available...
if nargin < 7
    
    if (strcmp(TYPE,'NORMAL'))
        
        l1 = legend(cell_s{4:5}, 'Location', 'NorthEast');
    else
        
        l1 = legend([p1, p3], cell_s{4:5}, 'Location', 'NorthEast');
    end
else
    if (strcmp(TYPE,'NORMAL'))
        
        l1 = legend(cell_s{4:6}, 'Location', 'NorthEast');
    else
        
        l1 = legend([p1, p2, p3], cell_s{4:6}, 'Location', 'NorthEast');
    end
end

%%

t1 = title (cell_s{1});
x1 = xlabel(cell_s{2});
y1 = ylabel(cell_s{3});

set(t1,'FontSize', font_title);
set(x1,'FontSize', font_label);
set(y1,'FontSize', font_label);
set(l1,'FontSize', font_legend);
set(gca, 'YTickMode', 'auto', 'FontSize', font_tick);

