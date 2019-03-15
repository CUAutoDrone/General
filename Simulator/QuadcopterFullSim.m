clearvars;

%Simulation Parameters
start_time = 0; %sec
end_time = 60; %sec
dt = 0.01; %sec - set this as sensor refresh rate
position_gain = 0.7; % Use to tune the position controller
plotting = 1; %turns on plotting

%helper quantities (dont worry about this)
times = start_time:dt:end_time;
N = numel(times);
step = 1; %timestep counter

%Drone Physical Properties
%Moment of Inertia Tensor, kg m^2
I = [8.1e-2 0 0; 
    0 8.1e-1 0; 
    0 0 0.142];
m = 1.3; %kg
g = 9.806;
R = 0.175;
L = R/sqrt(2); %m
%propellor drag coefficient
rho = 1.225; %kg/m^3 density of air
Cp = 0.025;
b=1e-7;
k = 3.83e-6;

max_thrust = m*g/2;


%initial conditions
theta = [deg2rad(0);deg2rad(0);0]; %initial angular position, in euler
omega = [0;0;0]; %some disturbance in angular velocity
q = eulertoq(theta); %calculates initial quaternion
p = [0;0;0]; %initial position
v = [0;0;0]; %initial velocity

%desired state
pref = [100;-30;10]; %desired position
vref = [0;0;0]; %desired velocity

%Plot Parameters - ignore
euler_state = zeros(3,N);
controleuler_state = zeros(3,N);
pos_state = zeros(3,N);
control = zeros(3,N);
motors = zeros(4,N);

[K,Kpos] = QuadcopterCombLQR(); %calculates LQR gain matrices

%step through sim
for time = start_time:dt:end_time
    euler_state(1:3, step) = rad2deg(theta);
    
    % find virtual target point at most 10m from drone.
    disp = pref - p;
    if norm(disp) > 10
        vtp = p + 10 * disp / norm(disp);
    else
        vtp = pref;
    end
    
    %calculate state error with respect to virtual target
    [thrust, qr] = lqr_thrust_attitude(position_gain * Kpos, vtp, p, vref, v, g, m);
    
    %establish control bounds
    [q_bound,thr_bound] = boundReferenceAttitude(thrust,qr,2*m*g,80,80);
    controleuler_state(:,3) = rad2deg(qtoeuler(q_bound));
    
    %calculate attitude error
    qerr = quatmultiply(q_bound,quatinv(q));
    
    %calculate input torque from attitude
    tau_in = K*[qerr(2:4)';-omega];
    
    %bound input torque and thrust
    tm = control2motor(tau_in,thrust,L);
    total_torque_thrust = abs(tau_in(1))/L+abs(tau_in(2))/L;
    allowed_thrust = 2*m*g-total_torque_thrust;
    if thrust>allowed_thrust
        thrust = allowed_thrust;
    end
    
    %propagate
    [t_out,w_out,q_out,p_out,v_out] = QuadcopterIntegrator(q,omega,p,v,tau_in,thrust,[time,time+dt],100,I,m,g);
    omega = w_out(:,end);
    q = q_out(end,:);
    p = p_out(:,end);
    v = v_out(:,end);
    
    
    motors(:,step) = tm;

    theta = qtoeuler(q);
    control(1:3, step) = tau_in;
    pos_state(:,step) = p;
    step = step+1;
end

if plotting == 1
figure
hold on
plot(times,pos_state(1,:))
plot(times,pos_state(2,:))
plot(times,pos_state(3,:))
% plot(times,motors(1,:))
% plot(times,motors(2,:))
% plot(times,motors(3,:))
% plot(times,motors(4,:))
% plot(times,controleuler_state(1,:))
% plot(times,controleuler_state(2,:))
% plot(times,controleuler_state(3,:))
hold off
end

function [thrust,qr] = lqr_thrust_attitude(Kpos, pref, p, vref, v, g, m)

    perr = pref-p; 
    verr = vref-v;
    
    %calculate control thrust and attitude
    up = Kpos*[perr;verr]+[0;0;g];
    b = [0;0;1];
    qprime = [(dot(b,up)+norm(up)),-cross(b,up)'];
    qr = qprime/norm(qprime);
    thrust = norm(up)*m;
end

function tau = torque(wm, omega, I, L, b, k)
    tau_motors = [L*k*(-wm(1)^2-wm(2)^2+wm(3)^2+wm(4)^2);
        L*k*(wm(1)^2-wm(2)^2-wm(3)^2+wm(4)^2);
        b*(-wm(1)^2+wm(2)^2-wm(3)^2+wm(4)^2)];
    tau = tau_motors;
end

function [bounded_q,bounded_thrust] = boundReferenceAttitude(thrust,qin,max_thrust,max_roll,max_pitch)
    euler = rad2deg(qtoeuler(qin));
    if abs(euler(1))>max_roll
        euler(1) = max_roll*sign(euler(1));
    end
    if abs(euler(2))>max_pitch
        euler(2) = max_pitch*sign(euler(2));
    end
    bounded_q = eulertoq(deg2rad(euler));
    
    if thrust>max_thrust
        bounded_thrust = max_thrust;
    else
        bounded_thrust = thrust;
    end
end

function tm = control2motor(torque,thrust,L)
    tm1 = 0;
    tm2 = 0;
    tm3 = 0;
    tm4 = 0;
    if torque(1)>0
        tm3 = tm3+torque(1)/2/L;
        tm4 = tm4+torque(1)/2/L;
    else
        tm1 = tm1-torque(1)/2/L;
        tm2 = tm2-torque(1)/2/L;
    end
    if torque(2)>0
        tm1 = tm1+torque(2)/2/L;
        tm4 = tm4+torque(2)/2/L;
    else
        tm3 = tm3-torque(2)/2/L;
        tm2 = tm2-torque(2)/2/L;
    end
    total_torque_thrust = abs(torque(1))/L+abs(torque(2))/L;
    res_thrust = thrust-total_torque_thrust;
    tm1 = tm1+res_thrust/4;
    tm2 = tm2+res_thrust/4;
    tm3 = tm3+res_thrust/4;
    tm4 = tm4+res_thrust/4;
    tm = [tm1;tm2;tm3;tm4];
end

function euler_state = qtoeuler(q)
% pitch = atan2(2*q(1)*q(3)-2*q(2)*q(4),q(1)^2+q(2)^2-q(3)^2-q(4)^2);
% yaw = asin(2*q(1)*q(4)+2*q(2)*q(3));
% roll = atan2(2*q(1)*q(2)-2*q(3)*q(4),q(1)^2-q(2)^2+q(3)^2-q(4)^2);

roll = atan2(2*(q(1)*q(2)+q(3)*q(4)),1-2*(q(2)^2+q(3)^2));
pitch = asin(2*(q(1)*q(3)-q(4)*q(2)));
yaw = atan2(2*(q(1)*q(4)+q(2)*q(3)),1-2*(q(3)^2+q(4)^2));
euler_state = [roll;pitch;yaw];
end

function q = eulertoq(euler)
r = euler(1);
p = euler(2);
y = euler(3);
q0 = cos(r/2)*cos(p/2)*cos(y/2)+sin(r/2)*sin(p/2)*sin(y/2);
q1 = sin(r/2)*cos(p/2)*cos(y/2)-cos(r/2)*sin(p/2)*sin(y/2);
q2 = cos(r/2)*sin(p/2)*cos(y/2)+sin(r/2)*cos(p/2)*sin(y/2);
q3 = cos(r/2)*cos(p/2)*sin(y/2)-sin(r/2)*sin(p/2)*cos(y/2);
q = [q0 q1 q2 q3];
end