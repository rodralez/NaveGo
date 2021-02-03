function line_ctr = file_line_counter(filename)
% file_line_counter: counts the lines (rows) in a text file.
%
% INPUT
%   filename: name or complete path to the text file under analysis
%   (string).
%
% OUTPUT
%   line_ctr: number of lines (rows) in the file.
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
% Version: 001
% Date:    2020/11/03
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

fid = fopen(filename, 'r');

if fid == -1
    error('file_line_counter: %s is not found', fid)
end

line_ctr = 0;

while ~feof(fid)
    fgetl(fid);
    line_ctr = line_ctr + 1;
end

% Set pointer back to the beginning of file
% fseek(fid,0,'bof');

fclose(fid);

fprintf('file_line_counter: lines in %s: %d', file, line_ctr));

end
