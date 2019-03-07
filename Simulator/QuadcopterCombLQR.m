function [K,K_pos] = QuadcopterCombLQR()
I = [8.1e-2 0 0; 
    0 8.1e-1 0; 
    0 0 0.142];
Ixx = I(1,1);
Iyy = I(2,2);
Izz = I(3,3);

%Assuming square quadrotor, distance between motor and axis
R = 0.175;
L = R/sqrt(2); %m
%propellor drag coefficient
rho = 1.225; %kg/m^3
Cp = 0.025;
%b = 0.5*rho*R^3*0.025;
b=1e-7;

%propellor torque coefficient
w_max = 7330;
%w_max = 5000;
%k = 1.862e-8; %kg s^2
k = 1.862e-6;

g = 9.806;
m = 1.3; %kg

A = [0 0 0 1/2 0 0;
    0 0 0 0 1/2 0;
    0 0 0 0 0 1/2;
    zeros(3) zeros(3)];

B = [0 0 0;
    0 0 0;
    0 0 0;
    1/Ixx 0 0;
    0 1/Iyy 0;
    0 0 1/Izz];

C = [1 0 0 0 0 0;
   0 1 0 0 0 0;
   0 0 1 0 0 0];

D = zeros(3,3);

sys = ss(A,B,C,D);

Q = 180*(C'*C);
R = 1*eye(3);
N = zeros(6,3);

[K,S,e] = lqr(sys,Q,R,N);

A_pos = [0 0 0 1 0 0;
        0 0 0 0 1 0;
        0 0 0 0 0 1;
        0 0 0 0 0 0;
        0 0 0 0 0 0;
        0 0 0 0 0 0];
    
B_pos = [0 0 0;
        0 0 0;
        0 0 0;
        1 0 0;
        0 1 0;
        0 0 1];
    
C_pos = [1 0 0 0 0 0;
        0 1 0 0 0 0;
        0 0 1 0 0 0];

D_pos = zeros(3,3);
sys_pos = ss(A_pos,B_pos,C_pos,D_pos);

Q_pos = 2*(C_pos'*C_pos);
R_pos = 1*eye(3);
N_pos = zeros(6,3);

[K_pos,S,e] = lqr(sys_pos,Q_pos,R_pos,N_pos)

end