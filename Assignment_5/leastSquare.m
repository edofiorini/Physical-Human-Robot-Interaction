function [beta] = leastSquare(position, velocity, voltage)

       Y = voltage';
       X = [velocity', position'];
       
       beta = inv(X'*X)*X'*Y;
       
end