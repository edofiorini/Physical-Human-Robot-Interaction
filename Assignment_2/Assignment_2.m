%% Simulink parameters
%four channel bilateral teleoperation architecture with local force feedback

clear all; close all; clc;

% Input function parameter (sin or step with low pass filter)
A = 1;

% Low pass frequency cuff off
Fip = 1;
% Sin frequency
Fc = 1; 

% Human intention controller (PI)
Ph = 20000;
Dh = 1000;

% Master controller
Bm = 0.8;
Km = 1;

% Slave controller
Bs = 0.8*4;
Ks = 4;

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
Be = 0; %100;
Ke = 200;

% Local force feedback
Cmf = 1;
Csf = 1;
Ts = 0.001;

% High frequency pole
tau = 10000;

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
