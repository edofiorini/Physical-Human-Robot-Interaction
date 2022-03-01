% Sample parameters for four channel bilateral teleoperation
clear all;
close all;

% Input function parameter (sin or step with low pass filter)
Amp = 1;

% Low pass frequency cuff off
Fip = 10;
% Sin frequency
Fc = 0.5; 

% Human intention controller (PI)
Ih = 1;%5; %10000;
Ph = 2;%5; %200;

% Master controller (PI)
Bm = 10;%100;
Km = 15;%50;

% Slave controller (PI)
Bs = 15;%100;
Ks = 10;%80;

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
Jh = 0.05;% 0;
Bh = 1.5;%1.5;
Kh = 200; %1; %2000;

% Environment impedance parameters
Je = 0;
Be = 10; %100;
Ke = 200;%10;%200;

Ts = 0.001;

% High frequency pole
tau = 100000;

b = 1;
delay = 1;