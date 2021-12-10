function [x_k] = kalmanPredictorSteadyState(y_k, A, C, R, Q)

    
        %%%Notation:
        % x_k_1   is   x_k+1|k
        % x_k     is   x_k|k-1
        % y_k_1   is   y_k
        %%%
        
        x_k = zeros(3, size(y_k, 2));
  

        % Algebaric Riccati equation
        [P] = idare(A',C',Q,R, [], eye(3,3));  
        
        % Kalman gain
        K_inf = A*P*C'*inv(C*P*C' + R);
         
        for i = 2: size(y_k, 2)

            x_k_1 = A*x_k(:,i-1) + K_inf*(y_k(i) - C*A*x_k(:,i-1));
            x_k(:,i) = x_k_1;
            
        end
        
end

