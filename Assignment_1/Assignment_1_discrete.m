%% Simulink parameters
% four channel bilateral teleoperation architecture

% TODO settare i parametri giusti
% KALMAN toglie rumore sulle forze, ma peggi oin riferimento -- second set
% FIltro meglio in riferimento, ma peggio sul romore --- first set param
clear all; close all; clc;

% Input function parameter (sin or step with low pass filter)
Amp = 1;

% Low pass frequency cuff off
Fip = 10;
% Sin frequency
Fc = 1; 

% Human intention controller (PI)
Ph = 13000;%4000;
Dh = 700;%1000

% Ph = 4000;
% Dh = 1000;

% Master controller
Bm = 100;%0.8;%*40;
Km = 0;%1;%*40;

% Bm = 0.8;
% Km = 0;

% Slave controller
Bs = 100;%0.8*4;%*4;
Ks = 0;%4;%*4;

% Bs = 0.8*4;
% Ks = 4;%*4;

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
Jh = 1;
Bh = 70;
Kh = 2000;

% Environment impedance parameters
Je = 0;
Be = 0; %100;
Ke = 200;

Ts = 0.001;

% High frequency pole
tau = 10000;

% Kalman
A = [1 Ts
    0 1];
B = [Ts^2/2;Ts];
x0 = [0 0];
C = [1 0];
q = 10000;
R = 1;
Q = q*B*B';

s = tf('s');
Zm = Mm*s+Dm;
Zs = Ms*s+Ds;
Cm = (Bm*s+Km)/s;
Cs = (Bs*s+Ks)/s;
C4 = -(Mm*s^2+(Bm+Dm)*s+Km)/s;
C2 = 1;
C1 = (Ms*s^2+(Bs+Ds)*s+Ks)/s;
C3 = 1;
D = 1/(C1+C3*Zm+C3*Cm);

H11 = (Zm+Cm)*D*(Zs+Cs-C3*C4)+C4;
H12 = -(Zm+Cm)*D*(1-C3*C2)-C2;
H21 = minreal(D*(Zs+Cs-C3*C4));
H22 = -D*(1-C3*C2);

Zwidth = (H12*H21 - H11*H22) / (H22*H21);

% G = w_n^2/(s^2+2*w_n*xi*s+w_n^2)