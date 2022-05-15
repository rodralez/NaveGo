function dcen = diff_central (x, y)
% diff_central: central derivative.
%
% INPUT
%   x: Nx1 independent variable.
%   y: Nx1 dependent variable.
%
% OUTPUT
%   dcen: (N-2)x1 central derivative.
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
% 	https://en.wikipedia.org/wiki/Finite_difference
%
% Version: 001
% Date:    2021/02/04
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

[n,m] = size(x);

if (m > n) % If x is a row vector
    
    % Transpose
    x = x';
end

[n,m] = size(y);

if (m > n) % If y is a row vector
    
    % Transpose
    y = y';
    n = m;
end

% Derivative for even elements
dcen_even   = diff(y(1:2:end)) ./ diff(x(1:2:end));

% Derivative for odd elements
dcen_odd = diff(y(2:2:end)) ./ diff(x(2:2:end));

dcen = zeros(size(x));

if ( mod(n,2) == 0) % If n is even... 
    
    dcen (2:2:end-2) = dcen_even;
    dcen (3:2:end)   = dcen_odd;
    
else 				% If n is odd... 
    
    dcen (2:2:end)   = dcen_even;
    dcen (3:2:end-2) = dcen_odd;
end

% Two elements are lost, the first one and the last one
dcen = dcen (2:end-1);
