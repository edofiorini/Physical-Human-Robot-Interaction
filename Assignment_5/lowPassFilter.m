function [y] = lowPassFilter(u, Fc, Ts)

    % u is the input 
    % Fc is the cut frequency
    % Ts is the sampling time
    
    y = zeros(1, size(u,2));
    
    p = -2*pi*Fc;
    beta = exp(p*Ts);
    
    for i=2:size(u,2)
        
        y(i) = beta*y(i-1) + (1 - beta)*u(i);
    end
end

