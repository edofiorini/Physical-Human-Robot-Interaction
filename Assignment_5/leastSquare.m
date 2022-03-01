function [beta] = leastSquare(velocity, acceleration, voltage)

       Y = voltage';
       X = [acceleration', velocity'];
       
       beta = inv(X'*X)*X'*Y;
       
end