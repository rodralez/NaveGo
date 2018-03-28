function hL = dsplot(x, y, numPoints)

%DSPLOT Create down sampled plot.
%   This function creates a down sampled plot to improve the speed of
%   exploration (zoom, pan).
%
%   DSPLOT(X, Y) plots Y versus X by downsampling if there are large number
%   of elements. X and Y needs to obey the following:
%     1. X must be a monotonically increasing vector.
%     2. If Y is a vector, it must be the same size as X.
%     3. If Y is a matrix, one of the dimensions must line up with X.
%
%   DSPLOT(Y) plots the columns of Y versus their index.
%
%   hLine = DSPLOT(X, Y) returns the handles of the line. Note that the
%   lines may be downsampled, so they may not represent the full data set.
%
%   DSPLOT(X, Y, NUMPOINTS) or DSPLOT(Y, [], NUMPOINTS) specifies the
%   number of points (roughly) to display on the screen. The default is
%   50000 points (~390 kB doubles). NUMPOINTS can be a number greater than
%   500.
%
%   It is very likely that more points will be displayed than specified by
%   NUMPOINTS, because it will try to plot any outlier points in the range.
%   If the signal is stochastic or has a lot of sharp changes, there will
%   be more points on plotted on the screen.
%
%   The figure title (name) will indicate whether the plot shown is
%   downsampled or is the true representation.
%
%   The figure can be saved as a .fig file, which will include the actual
%   data. The figure can be reloaded and the actual data can be exported to
%   the base workspace via a menu.
%
%   Run the following examples and zoom/pan to see the performance.
%
%  Example 1: (with small details)
%   x  = linspace(0, 2*pi, 1000000);
%   y1 = sin(x)+.02*cos(200*x)+0.001*sin(2000*x)+0.0001*cos(20000*x);
%   dsplot(x,y1);title('Down Sampled');
%   % compare with
%   figure;plot(x,y1);title('Normal Plot');
%
%  Example 2: (with outlier points)
%   x  = linspace(0, 2*pi, 1000000);
%   y1 = sin(x) + .01*cos(200*x) + 0.001*sin(2000*x);
%   y2 = sin(x) + 0.3*cos(3*x)   + 0.001*randn(size(x));
%   y1([300000, 700000, 700001, 900000]) = [0, 1, -2, 0.5];
%   y2(300000:500000) = y2(300000:500000) + 1;
%   y2(500001:600000) = y2(500001:600000) - 1;
%   y2(800000) = 0;
%   dsplot(x, [y1;y2]);title('Down Sampled');
%   % compare with
%   figure;plot(x, [y1;y2]);title('Normal Plot');
%
%  See also PLOT.

%  Version:
%   v1.0 - first version (Aug 1, 2007)
%   v1.1 - added CreateFcn for the figure so that when the figure is saved
%          and re-loaded, the zooming and panning works. Also added a menu
%          item for saving out the original data back to the base
%          workspace. (Aug 10, 2007)
%
%  Jiro Doke
%  August 1, 2007

debugMode = false;

%--------------------------------------------------------------------------
% Error checking
error(nargchk(1, 3, nargin, 'struct'));
if nargin < 3
  % Number of points to show on the screen. It's quite possible that more
  % points will be displayed if there are outlier points
  numPoints = 50000;  % ~390 kB for doubles
end
if nargin == 1 || isempty(y)
  noXVar = true;
  y = x;
  x = [];
else
  noXVar = false;
end
myErrorCheck;
%--------------------------------------------------------------------------

if size(x, 2) > 1  % it's a row vector -> transpose
  x = x';
  y = y';
  varTranspose = true;
else
  varTranspose = false;
end

% Number of lines
numSignals = size(y, 2);

% If the number of lines is greater than the number of data points per
% line, it's possible that the user may have mistaken the matrix
% orientation.
if numSignals > size(y, 1)
  s = input(sprintf('Are you sure you want to plot %d lines? (y/n) ', ...
    numSignals), 's');
  if ~strcmpi(s, 'y')
    disp('Canceled. You may want to transpose the matrix.');
    if nargout == 1
      hL = [];
    end
    return;
  end
end

% Attempt to find outliers. Use a running average technique
filterWidth = ceil(min([50, length(x)/10])); % max window size of 50
a  = y - filter(ones(filterWidth,1)/filterWidth, 1, y);
[iOutliers, jOutliers] = find(abs(a - repmat(mean(a), size(a, 1), 1)) > ...
  repmat(4 * std(a), size(a, 1), 1));
clear a;

% Always create new figure because it messes around with zoom, pan,
% datacursors.
hFig    = figure;
figName = '';

% Create template plot using NaNs
hLine   = plot(NaN(2, numSignals), NaN(2, numSignals));
set(hLine, 'tag', 'dsplot_lines');

% Define CreateFcn for the figure
set(hFig, 'CreateFcn', @mycreatefcn);
mycreatefcn();

% Create menu for exporting data
hMenu = uimenu(hFig, 'Label', 'Data');
uimenu(hMenu, ...
  'Label'   , 'Export data to workspace.', ...
  'Callback', @myExportFcn);

% Update lines
updateLines([min(x), max(x)]);

% Deal with output argument
if nargout == 1
  hL = hLine;
end

%--------------------------------------------------------------------------
  function myExportFcn(varargin)
    % This callback allows for extracting the actual data from the figure.
    % This means that if you save this figure and load it back later, you
    % can get back the data.
    
    % Determine the variable name
    allVarNames = evalin('base', 'who');
    newVarName = genvarname('dsplotData', allVarNames);
    
    % X
    if ~noXVar
      if varTranspose
        dat.x = x';
      else
        dat.x = x;
      end
    end
    
    % Y
    if varTranspose
      dat.y = y';
    else
      dat.y = y;
    end
    
    assignin('base', newVarName, dat);
    
    msgbox(sprintf('Data saved to the base workspace as ''%s''.', ...
      newVarName), 'Saved', 'modal');
    
  end

%--------------------------------------------------------------------------
  function mycreatefcn(varargin)
    % This callback defines the custom zoom/pan functions. It is defined as
    % the CreateFcn of the figure, so it allows for saving and reloading of
    % the figure.

    if nargin > 0
      hFig = varargin{1};
    end
    hLine = findobj(hFig, 'type', 'axes');
    hLine(strmatch('legend', get(hLine, 'tag'))) = [];
    hLine = get(hLine, 'Children');
    
    % Create Zoom, Pan, Datacursor objects
    hZoom = zoom(hFig);
    hPan  = pan(hFig);
    hDc   = datacursormode(hFig);
    set(hZoom, 'ActionPostCallback', @mypostcallback);
    set(hPan , 'ActionPostCallback', @mypostcallback);
    set(hDc  , 'UpdateFcn'         , @myDCupdatefcn);

  end

%--------------------------------------------------------------------------
  function mypostcallback(obj, evd) %#ok
    % This callback that gets called when the mouse is released after
    % zooming or panning.

    % single or double-click
    switch get(hFig, 'SelectionType')
      case {'normal', 'alt'}
        updateLines(xlim(evd.Axes));

      case 'open'
        updateLines([min(x), max(x)]);

    end

  end

%--------------------------------------------------------------------------
  function updateLines(rng)
    % This helper function is for determining the points to plot on the
    % screen based on which portion is visible in the current limits.

    % find indeces inside the range
    id = find(x >= rng(1) & x <= rng(2));

    % if there are more points than we want
    if length(id) > numPoints / numSignals

      % see how many outlier points are in this range
      blah = iOutliers > id(1) & iOutliers < id(end);

      % determine indeces of points to plot. 
      idid = round(linspace(id(1), id(end), round(numPoints/numSignals)))';

      x2 = cell(numSignals, 1);
      y2 = x2;
      for iSignals = 1:numSignals
        % add outlier points
        ididid = unique([idid; iOutliers(blah & jOutliers == iSignals)]);
        x2{iSignals} = x(ididid);
        y2{iSignals} = y(ididid, iSignals);
      end

      if debugMode
        figName = ['downsampled - ', sprintf('%d, ', cellfun('length', y2))];
      else
        figName = 'downsampled';
      end

    else % no need to down sample
      figName = 'true';

      x2 = repmat({x(id)}, numSignals, 1);
      y2 = mat2cell(y(id, :), length(id), ones(1, numSignals))';

    end

    % Update plot
    set(hLine, {'xdata', 'ydata'} , [x2, y2]);
    set(hFig, 'Name', figName);

  end

%--------------------------------------------------------------------------
  function txt = myDCupdatefcn(empt, event_obj) %#ok
    % This function displays appropriate data cursor message based on the
    % display type

    pos = get(event_obj,'Position');
    switch figName
      case 'true'
        txt = {['X: ',num2str(pos(1))],...
          ['Y: ',num2str(pos(2))]};
      otherwise
        txt = {['X: ',num2str(pos(1))],...
          ['Y: ',num2str(pos(2))], ...
          'Warning: Downsampled', ...
          'May not be accurate'};
    end
  end

%--------------------------------------------------------------------------
  function myErrorCheck
    % Do some error checking on the input arguments.

    if ~isa(numPoints, 'double') || numel(numPoints) > 1 || numPoints < 500
      error('Third argument must be a scalar greater than 500');
    end
    if ~isnumeric(x) || ~isnumeric(y)
      error('Arguments must be numeric');
    end
    if length(size(x)) > 2 || length(size(y)) > 2
      error('Only 2-D data accepted');
    end
    
    % If only one input, create index vector X
    if isempty(x)
      if ismember(1, size(y))
        x = reshape(1:numel(y), size(y));
      else
        x = (1:size(y, 1))';
      end
    end
    
    if ~ismember(1, size(x))
      error('First argument has to be a vector');
    end
    if ~isequal(size(x, 1), size(y, 1)) && ~isequal(size(x, 2), size(y, 2))
      error('One of the dimensions of the two arguments must match');
    end
    if any(diff(x) <= 0)
      error('The first argument has to be a monotonically increasing vector');
    end
  end

end