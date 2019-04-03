function r = plot_cdf (data, pd, str_title)

N = length(data);
sig = pd.sigma;
mu = pd.mu;

x = linspace(min(data), max(data), N );
gauss_cdf = (normcdf(x, mu, sig))';

% Next, compute the empirical cumulative distribution function (ECDF) of the data. 
% This is simply a step function with a jump in cumulative probability, p, of 1/n at each data point, x.

x_sort = sort(data);
pn = ( (1:N) - 0.5)' ./ N;

r = rmse(gauss_cdf, pn);

% Plot the cdf
p1 = plot(x, gauss_cdf, '-.r',  'LineWidth', 2);
hold on
p2 = stairs(x_sort, pn,'-b', 'LineWidth', 2);
xlabel('x');
ylabel('Cumulative probability (p)');
legend([p1, p2], 'Reference CDF', 'Empirical CDF' )

grid
hold off


