function dcen = diff_central (x, y)

[n,m] = size(x);

if (m > n) % If x is a row vector
    
    % Transpose
    x = x';
    y = y';
    
    n = m;
end

% Derivative for even elements
dcen_par   = diff(y(1:2:end)) ./ diff(x(1:2:end));

% Derivative for odd elements
dcen_impar = diff(y(2:2:end)) ./ diff(x(2:2:end));

dcen = zeros(size(x));

if ( mod(n,2) == 0) % If n is even... 
    
    dcen (2:2:end-2) = dcen_par;
    dcen (3:2:end)   = dcen_impar;
    
else 				% If n is odd... 
    
    dcen (2:2:end)   = dcen_par;
    dcen (3:2:end-2) = dcen_impar;
end

% Two elements are lost, the first one and the last one
dcen = dcen (2:end-1);
