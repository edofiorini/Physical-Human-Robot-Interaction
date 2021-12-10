% Identify the parameters k and tau using the LS and the RLS on the DC motors data.

clear all; close all; clc;

% Import the data 
data = importdata('master_slave_1kHz.txt');
%data = importdata('master_slave_2kHz.txt');
%data = importdata('master_slave_500Hz.txt');

time = data.data(370:end,1)';
position_master = data.data(370:end,2)';
velocity_master = data.data(370:end,3)';
voltage_master = data.data(370:end,4)';

Ts = 0.001;

%velocity and acceleration discrete dynamics model
A = [ 1 Ts Ts^2/2
      0  1  Ts
      0  0   1     ];

C = [1 0 0];

q = 100000;
Q = q*[Ts^3/6; Ts^2/2; Ts]*[Ts^3/6; Ts^2/2; Ts]';
R = 1;%std(position_master);

% Initial conditions
x0 = [0; 0; 0 ];
P0 = [1e-4 0   0 
      0   1e-4 0 
      0    0   1e-4  ];

 % Estimate the velocity and acceleration to estimate the dc motor
 % parameters
[x_k_filter] = kalmanFilter(position_master, A, C, R, Q, x0, P0);

initialBeta = [0; 0];
initialP = [ 0.1 0
             0 0.1 ];
g = 0.005;

lambda = 0.99;

[beta_LS] = leastSquare(x_k_filter(2,:), x_k_filter(3,:), voltage_master);
[beta_RLS] = recursiveLeastSquare(x_k_filter(2,:), x_k_filter(3,:), voltage_master, initialBeta, initialP, lambda);
[beta_AA] = adaptiveAlgorithm(x_k_filter(2,:), x_k_filter(3,:), voltage_master, initialBeta, g, Ts);

% Least square prediction in trainig
prediction_LS = [x_k_filter(2,:)', x_k_filter(3,:)']*beta_LS;
figure(1)
plot(time,prediction_LS)
hold on
plot(time, lowPassFilter(voltage_master, 5, Ts))
title('Least Square');xlabel('Time'); ylabel('Voltage');
legend('Prediction', 'Dataset');

% Recursive Least square prediction in trainig
prediction_RLS = [x_k_filter(2,:)', x_k_filter(3,:)']*beta_RLS;
figure(2)
plot(time,prediction_RLS)
hold on
plot(time, lowPassFilter(voltage_master, 5, Ts))
title('Recursive Least Square');xlabel('Time'); ylabel('Voltage');
legend('Prediction', 'Dataset');

% Recursive Least square prediction in trainig
prediction_AA = [x_k_filter(2,:)', x_k_filter(3,:)']*beta_AA;
figure(3)
plot(time,prediction_AA)
hold on
plot(time, lowPassFilter(voltage_master, 5, Ts))
title('Adaptive Algorithm');xlabel('Time'); ylabel('Voltage');
legend('Prediction', 'Dataset');

figure(4)
plot(time, lowPassFilter(voltage_master, 5, Ts))
hold on
plot(time,prediction_LS)
hold on
plot(time,prediction_RLS)
hold on
plot(time,prediction_AA)
title('Prediction');xlabel('Time'); ylabel('Voltage');
legend('Dataset', 'Least Square', 'Recursive Least Square', 'Adaptive Algorithm');

% Finding tau and k
k_LS  = 1/beta_LS(2);
tau_LS = beta_LS(1)*k_LS;

k_RLS  = 1/beta_RLS(2);
tau_RLS = beta_RLS(1)*k_RLS;

k_AA  = 1/beta_AA(2);
tau_AA = beta_AA(1)*k_AA;