% Assignemnt_3/4

%%% Kalman filter and predictor for velocities and acceleration
%%% Kalman steady-state filter predictor for velocities and acceleration

clear all; close all; clc;

% Import the data 
data = importdata('master_slave_1kHz.txt');
%data = importdata('master_slave_2kHz.txt');
%data = importdata('master_slave_500Hz.txt');

time = data.data(370:end,1)';
position_master = data.data(370:end,2)';
velocity_master = data.data(370:end,3)';
 
Ts = 0.001;


%velocity and acceleration discrete dynamics model
A = [ 1 Ts Ts^2/2
      0  1  Ts
      0  0   1     ];

C = [1 0 0];

q = 100000;
Q = q*[Ts^3/6; Ts^2/2; Ts]*[Ts^3/6; Ts^2/2; Ts]';
R = 1;

% Initial conditions
x0 = [0; 0; 0 ];
P0 = [1e-4 0   0 
      0   1e-4 0 
      0    0   1e-4  ];
  
[x_k_filter] = kalmanFilter(position_master, A, C, R, Q, x0, P0);
[x_k_predictor] = kalmanPredictor(position_master, A, C, R, Q, x0, P0);
[velocity_Euler] = eulerApproximationVelocity(position_master, Ts);
[x_k_smoothing] = kalmanSmoother(position_master, A, C, R, Q, x0, P0);

[x_k_filter_SteadyState] = kalmanFilterSteadyState(position_master, A, C, R, Q);
[x_k_predictor_SteadyState] = kalmanPredictorSteadyState(position_master, A, C, R, Q);

% Low pass filter 
Fc = 5;
[velocity_Euler_lpf] = lowPassFilter(velocity_Euler, Fc, Ts);

% Position
figure(1);
plot(time, position_master);
hold on
plot(time, x_k_filter(1,:));
hold on
plot(time, x_k_predictor(1,:));
hold on
plot(time, x_k_filter_SteadyState(1,:));
hold on
plot(time, x_k_predictor_SteadyState(1,:));
hold on
plot(time, x_k_smoothing(1,:));
legend('Real', 'Kalman Filter', 'Kalman Predictor', 'Kalman Steady-state Filter', 'Kalman Steady-state Predictor', 'Kalman Smoother' );
title('Position');xlabel('Time'); ylabel('Position');

% Velocity
figure(2);
plot(time, velocity_Euler_lpf);
hold on
plot(time, velocity_master);
hold on
plot(time, x_k_filter(2,:));
hold on
plot(time, x_k_predictor(2,:));
hold on
plot(time, x_k_smoothing(2,:));
hold on
plot(time,x_k_filter_SteadyState(2,:));
hold on
plot(time,x_k_predictor_SteadyState(2,:));
legend('Euler Approximation lpf', 'Dataset', 'Kalman Filter', 'Kalman Predictor', 'Kalman Smoother','Kalman Steady-state Filter', 'Kalman Steady-state Predictor' );
title('Velocity');xlabel('Time'); ylabel('Velocity');

% Acceleration
figure(3);
plot(time, x_k_filter(3,:));
hold on
plot(time, x_k_predictor(3,:));
hold on
plot(time, x_k_smoothing(3,:));
hold on
plot(time,x_k_filter_SteadyState(3,:));
hold on
plot(time,x_k_predictor_SteadyState(3,:));
legend('Kalman Filter', 'Kalman Predictor', 'Kalman Smoother', 'Steady-state Filter','Steady-state Predictor');
title('Acceleration');xlabel('Time'); ylabel('Acceleration');



 


