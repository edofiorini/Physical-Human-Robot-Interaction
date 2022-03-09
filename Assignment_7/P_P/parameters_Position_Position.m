% Sample parameters for four channel bilateral teleoperation
clear all;
close all;

% Input function parameter (sin or step with low pass filter)
Amp = 1;%0.5;

% Low pass frequency cuff off
Fip = 100;%1;
% Sin frequency
Fc = 0.5; %1; 

% Human intention controller (PI)
Ih = 2; %10000;
Ph = 5; %200;


% Master controller (PI)
Bm = 250;
Km = 80;

% Slave controller
Bs = 200;
Ks = 80;%40;

% Intertia of robot dynamics
Mm = 0.5;
Ms = 2;

% Make the real pole with re<0 to change the inertia of the robot
% dynamics verify this value by adding them to the simulink model
% Dm = 5;
% Ds = 10;
Dm = 0;
Ds = 0;

% Human impedance parameters
Jh = 0;%0.05;
Bh = 1.5;
Kh = 1; %2000;

% Environment impedance parameters
Je = 0;
Be = 10; %100;
Ke = 200;%10;%200;

Ts = 0.001;

% High frequency pole
tau = 100000;
beta =  0.3;
H_D = 1;
alpha = 0.1;
delay = 1;

H_init_m = 10;
H_init_s = 0;
