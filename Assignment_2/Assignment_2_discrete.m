%% Simulink parameters
%four channel bilateral teleoperation architecture with local force feedback

% first step -- with noise but better in position
%second step -- without noise, but bad in position -- KALMAN
clear all; close all; clc;

% Input function parameter (sin or step with low pass filter)
Amp = 1;

% Low pass frequency cuff off
Fip = 1;
% Sin frequency
Fc = 1; 

% Human intention controller (PI)
Ph = 20000;
Dh = 100;

% Ph = 200;
% Dh = 100;
% Master controller PI velocity
Bm = 0.8*180; % P 
Km = 1*180;   % I

% Bm = 0.8*30;
% Km = 1*30;
% Slave controller
Bs = 0.8*4*2;
Ks = 4*2;

% Bs = 0.8*4;
% Ks = 4;
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
Jh = 0.5;
Bh = 70;
Kh = 2000;

% Environment impedance parameters
Je = 0;
Be = 10; %100;
Ke = 20; % Change it to have a better response

% Local force feedback
Cmf = 0.6;
Csf = 0.6;
Ts = 0.001;

% High frequency pole
tau = 10000;

% Kalman
A = [1 Ts
    0 1];
B = [Ts^2/2;Ts];
x0 = [0 0];
C = [1 0];
q = 100;
q = 100000;
R = 1;
Q = q*B*B';

s = tf('s');
Zm = Mm*s+Dm;
Zs = Ms*s+Ds;
Cm = (Bm*s+Km)/s;
Cs = (Bs*s+Ks)/s;
C4 = -(Mm*s^2+(Bm+Dm)*s+Km)/s;
C2 = 1 + Cmf;
C1 = (Ms*s^2+(Bs+Ds)*s+Ks)/s;
C3 = 1 + Csf;

Zcm = (Mm*s^2+(Bm+Dm)*s+Km)/s;
Zcs = (Ms*s^2+(Bs+Ds)*s+Ks)/s;

H11 = (Zcm*Zcs + C1*C4)/((1+Cmf)*Zcs - C3*C4);
H12 = (C2*Zcs - C4*(1+ Csf))/ ((1 + Cmf)*Zcs - C3*C4);
H21 = -((C3*Zcm + C1*(1 + Cmf))/((1 + Cmf)*Zcs - C3*C4));
H22 = ((1+ Csf)*(1+Cmf) - C2*C3)/((1+Cmf)*Zcs - C3*C4);
