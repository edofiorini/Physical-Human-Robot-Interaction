function [beta, pred] = recursiveLeastSquare(velocity, acceleration, voltage, initialBeta, initialP, lambda)
        
       yk = voltage';
       xk = [acceleration', velocity'];
       
       P_previous = initialP;
      
       beta_previous = P_previous*xk(1,:)'*yk(1);
       pred = zeros(1,size(velocity,2));
       for i = 2:size(velocity,2)
           
            e = yk(i) - xk(i,:)*beta_previous;
            Pk = (P_previous-(P_previous*xk(i,:)'*xk(i,:)*P_previous)/(lambda+xk(i,:)*P_previous*xk(i,:)'))/lambda;
            %Pk = 1/lambda*(P_previous - (P_previous*xk(i,:)'*xk(i,:)*P_previous)/(lambda + xk(i,:)*P_previous*xk(i,:)'));
            Kk = Pk*xk(i,:)';
       
            beta = beta_previous + Kk*e;
            pred(i) = xk(i,:)*beta;
            beta_previous = beta;
            P_previous = Pk;
            
       end
       
end

