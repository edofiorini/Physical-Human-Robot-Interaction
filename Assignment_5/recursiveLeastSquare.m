function [beta] = recursiveLeastSquare(position, velocity, voltage, initialBeta, initialP, lambda)
        
       yk = voltage';
       xk = [velocity', position'];
       
       beta_previous = initialBeta;
       P_previous = initialP;
      
       for i = 2:size(position,2)
           
            e = yk(i) - xk(i,:)*beta_previous;
            Pk = 1/lambda*(P_previous - (P_previous*xk(i,:)'*xk(i,:)*P_previous)/(lambda + xk(i,:)*P_previous*xk(i,:)'));
            Kk = Pk*xk(i,:)';
       
            beta = beta_previous + Kk*e;
            beta_previous = beta;
            P_previous = Pk;
            
       end
       
end

