function plot_histogram (samples, pd)

N = length(samples);

%% REFERENCE

x = linspace(min(samples), max(samples), N );
gauss_bell = pdf(pd, x);


%% STATISTIC ANALYSIS

ESPI = 1E-3;

pd_mean = mean(samples);
pd_median = median(samples);

idx1 = find( x >= pd_mean - ESPI   & x < pd_mean + ESPI );
idx2 = find( x >= pd_median - ESPI & x < pd_median + ESPI );

if ( isempty(idx1) || isempty(idx2) )
    error('plot_histogram: no match for mean or median')
end

idx1 = idx1( ceil(end/2) );
idx2 = idx2( ceil(end/2) );

%% PLOT

bins = 100;

% Plot histogram from data
h0 = histogram(samples, bins, 'Normalization', 'pdf', 'FaceColor', [.9 .9 .9]);
hold on

% Plot the reference pdf
p0 = plot(x, gauss_bell, '-',  'LineWidth', 2);

% Plot lines
y = gauss_bell (idx1);
l1 = line( [pd_mean, pd_mean] , [0, y], 'Color', [0 0.4470 0.7410], 'LineWidth', 2, 'LineStyle','-.');
y = gauss_bell (idx2);
l2 = line( [pd_median, pd_median] , [0, y], 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 2, 'LineStyle','-.' );

legend([p0, l1, l2], 'Reference PDF', 'Mean', 'Median')

grid on
hold off

end