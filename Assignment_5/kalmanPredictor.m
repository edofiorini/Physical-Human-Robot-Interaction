function [x_k] = kalmanPredictor(y_k, A, C, R, Q, x0, P0)

    
        %%%Notation:
        % x_k_1    is     x_k+1|k
        % x_k      is     x_k|k-1
        % y_k_1    is     y_k
        % P_k_1    is     P_k+1|k
        % P_k      is     P_k|k-1
        %%%
        
        x_k = zeros(size(x0,1), size(y_k, 2));
  
        % Initial condition
        x_k(:,1) = x0;
        P_k = P0;
        
        for i = 2: size(y_k, 2)
            
            % Riccati equation
            P_k_1 = A*P_k*A' - A*P_k*C'*inv(C*P_k*C' + R)*C*P_k*A' + Q;
            
            % Kalman gain
            K_k = A*P_k*C'*inv(C*P_k*C' + R);
            
            x_k_1 = A*x_k(:,i-1) + K_k*(y_k(i) - C*A*x_k(:,i-1));
            x_k(:,i) = x_k_1;
            P_k = P_k_1;
            
        end
        
end

