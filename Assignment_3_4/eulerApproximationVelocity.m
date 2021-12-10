function [velocity] = eulerApproximationVelocity(position, Ts)
    
    velocity = zeros(1, size(position, 2));
    
    for i = 2:size(position, 2)
        
        velocity(:,i) = (position(:,i) - position(:,i-1)) / Ts; 
    end
    
end

