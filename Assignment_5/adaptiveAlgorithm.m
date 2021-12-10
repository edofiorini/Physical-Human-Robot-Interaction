function [beta] = adaptiveAlgorithm(position, velocity, voltage, initialBeta, g, Ts)

       yk = voltage';
       xk = [velocity', position'];
       
       beta_previous = initialBeta;
       
       for i = 2:size(position,2)
           
            e = yk(i) - xk(i,:)*beta_previous;
           
            beta = beta_previous + Ts*g*xk(i,:)'*e;
            beta_previous = beta;
         
       end


end