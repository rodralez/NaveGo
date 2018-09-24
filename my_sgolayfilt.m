function signal_f = my_sgolayfilt (signal)
% my_sgolayfilt: Savitzky-Golay Filtering with variable frame length.
%
% INPUT:
%		signal: time serie(s) to be filter.
%
% OUTPUT:
%		signal_f: filtered signal, same order of signal.
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
%   MATLAB help for sgolayfilt function.
%
% Version: 001
% Date:    2018/09/19
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

signal_size = max(size(signal));

sgo_framelen = floor( signal_size / 5);

if (sgo_framelen > 21)
    sgo_framelen = 21;
end

if ( mod(sgo_framelen,2) == 0 )
    sgo_framelen = sgo_framelen +1;
end

sgo_order = ceil( sgo_framelen / 2);

signal_f = sgolayfilt(signal, sgo_order, sgo_framelen);

end
