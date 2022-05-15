function line_ctr = line_counter(file, verbose)
% line_counter: counter the number of lines in file.
%
% INPUT
%   file: file name (string).
%   verbose: 0, silence output.
%            1, verbose output.
%
% OUTPUT
%   line_ctr: number of lines in file.
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
% Date:    2020/11/25
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

if nargin < 2, verbose = 0; end

fid = fopen(file, 'r');

if fid == -1
    error('line_counter: %s not found', file)
else
    if verbose == 1
        fprintf('line_counter: processing %s...\n', file)
    end
end

line_ctr = 0;

while ~feof(fid)
    
    fgets(fid);
    line_ctr = line_ctr + 1;
    
    if verbose == 1
        % Print a dot on console every 10,000 executions
        if (mod(line_ctr, 10000) == 0), fprintf('. ');  end
        % Print a return on console every 200,000 executions
        if (mod(line_ctr, 200000) == 0), fprintf('\n'); end
    end
end

fclose(fid);

if verbose == 1
    fprintf('\nline_counter: %d lines in %s\n', line_ctr, file);
end

end
