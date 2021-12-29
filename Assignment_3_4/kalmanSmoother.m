function [x_k_s] = kalmanSmoother(y_k_1, A, C, R, Q, x0, P0)

    
        %%%Notation:
        %%% FORWARD
        % x_k_1_f      is              x_k+1|k+1_forward
        % x_k_f        is              x_k|k_forward
        % y_k_1        is              y_k+1
        % P_k_f        is              P_k|k
        % P_k_1_f      is              P_k+1|k_forward
        % P_k          is              P_k|k-1_ forward
        %%%
        %%% BACKWARD
        %%%
        % x_k_s        is              x_k|N_smoothing
        % P_k_1_N      is              P_k+1|N
        %%%
  
        
        % FORWARD STEP
        
        x_k_f = zeros(size(x0,1), size(y_k_1, 2));
        
        % Initial condition
        
        x_k_f(:,1) = x0;
        
        data_k  = struct('P_k_f', {P0}, 'P_k_1_f', {P0});
        data = [data_k];
        
        for i = 2: size(y_k_1, 2)
            
            P_k_f = data(i-1).P_k_1_f - data(i-1).P_k_1_f*C'*inv(C*data(i-1).P_k_1_f*C' + R)*C*data(i-1).P_k_1_f; 
            
           
            % Riccati equation
            P_k_1_f = A*data(i-1).P_k_1_f*A' - A*data(i-1).P_k_1_f*C'*inv(C*data(i-1).P_k_1_f*C' + R)*C*data(i-1).P_k_1_f*A' + Q;
            
            data_k  = struct('P_k_f', {P_k_f}, 'P_k_1_f', {P_k_1_f});
            data = [data, data_k];
            
            % Kalman gain
            K_k_1 = P_k_1_f*C'*inv(C*P_k_1_f*C' + R);
       
            x_k_1_f = A*x_k_f(:,i-1) + K_k_1*(y_k_1(i) - C*A*x_k_f(:,i-1));
            x_k_f(:,i) = x_k_1_f;
            
        end
        
        [x_k_predictor_f] = kalmanPredictor(y_k_1, A, C, R, Q, x0, P0);
        
        % BACKWARD STEP
 
        
        x_k_s = zeros(size(x0,1), size(y_k_1, 2));
  
        % Initial condition
        
        x_k_s(:,end) = x_k_f(:,end);
        P_k_1_N = data(end).P_k_f;
        
        
        for i = size(y_k_1, 2) - 1:-1:1
                       
            % Problem in this inversion matrix
            K_k = data(i).P_k_f*A'*inv(data(i).P_k_1_f);
            
            % Smoothing state 
            x_k_s(:,i) = x_k_f(:,i) + K_k*(x_k_s(:,i+1) - x_k_predictor_f(:,i+1));
            
            % Smoothing covariance
            P_k = data(i).P_k_f + K_k*(P_k_1_N - data(i).P_k_1_f );
            P_k_1_N = P_k;
            
        end
end

