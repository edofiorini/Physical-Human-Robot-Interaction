function [beta] = adaptiveAlgorithm(velocity, acceleration, voltage, initialBeta, g, Ts)

       yk = voltage';
       xk = [acceleration', velocity'];
       
       beta_previous = initialBeta;
       
       for i = 2:size(velocity,2)
           
            e = yk(i) - xk(i,:)*beta_previous;
           
            beta = beta_previous + Ts*g*xk(i,:)'*e;
            beta_previous = beta;
         
       end


end