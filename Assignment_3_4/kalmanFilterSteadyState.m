function [x_k] = kalmanFilterSteadyState(y_k_1, A, C, R, Q)

    
        %%%Notation:
        % x_k_1    is   x_k+1|k+1
        % x_k      is   x_k|k
        % y_k_1    is   y_k+1
        %%%
        
        x_k = zeros(3, size(y_k_1, 2));
        
        % Algebraic Riccati equation
        [P] = idare(A',C',Q,R, [], eye(3,3)); 
        
        % Kalman gain
        K_inf = P*C'*inv(C*P*C' + R);
        
        for i = 2: size(y_k_1, 2)
            
            x_k_1 = A*x_k(:,i-1) + K_inf*(y_k_1(i) - C*A*x_k(:,i-1));
            x_k(:,i) = x_k_1;
            
        end
        
end

