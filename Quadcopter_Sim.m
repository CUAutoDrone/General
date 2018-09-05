clear all;
%Simulation Parameters
start_time = 0; %sec
end_time = 5; %sec
dt = 0.005; %sec



%helper quantities
times = start_time:dt:end_time;
N = numel(times);
step = 0; %timestep counter
sum = [0;0;0]; %integral summer

%Drone Physical Properties
I = [1 0 0; 0 1 0; 0 0 1];
L = 1;
b = 1;
k = 100;

%Controller properties
Kp = .9;
Ki = 1;
Kd = 1.5;

%initial conditions
x = [0; 0; 10]; %initial position, m
xdot = zeros(3,1); %initial velocity, m/s
theta = [deg2rad(10);0;0]; %initial angular position, in 3-2-1 euler
omega = [0;0;0]; %some disturbance in angular velocity

%desired angle
control_r = [0;0;0;0];
theta_r = [control_r(2);control_r(3);control_r(4)];
throttle = control_r(1);

%Transformation Stuff
ctrl_to_uc = [1 -1 1 -1;
            1 -1 -1 1;
            1 1 -1 -1;
            1 1 1 1];
km = 1;

%Plot Parameters
states = rad2deg(theta);


%step through sim
for time = start_time:dt:end_time
    err = theta_r-theta;
    sum = sum+err;
    
    if step == 0
        u = Kp*err+Ki*dt*sum;
    else
        u = Kp*err+Ki*dt*sum+Kd/dt*(err-prev_err);
    end
    prev_err = err;
    
    ctrl = [throttle;
            (u(1));
            (u(2));
            (u(3));];
    uc = ctrl_to_uc*ctrl;
    wm = km*ZerotoOne(uc);
    
    state = [theta(1);theta(2);theta(3);omega(1);omega(2);omega(3)];
    [t_out,y] = ode45(@(t,y) propagator(t,y,wm,I,L,b,k),[time,time+dt],state);
    theta(1) = y(end,1);
    theta(2) = y(end,2);
    theta(3) = y(end,3);
    omega(1) = y(end,4);
    omega(2) = y(end,5);
    omega(3) = y(end,6);
    
    plot(t_out,y(:,1))
    
    states = [states rad2deg(theta)];
    %disp(rad2deg(theta))
    step = step+1;
       
end

plot(times,(states(1,2:end)))


function state_dot = propagator(t,state,wm,I,L,b,k)
    omega = [state(4);state(5);state(6)];
    theta = [state(1);state(2);state(3)];
    thetadot = omega_to_thetadot(omega,theta);
    omegadot = angular_acceleration(wm,omega, I,L,b,k);
    
    state_dot = [thetadot(1);
                thetadot(2);
                thetadot(3);
                omegadot(1);
                omegadot(2);
                omegadot(3)];
                
end

function omegadot = angular_acceleration(wm, omega, I, L, b, k)
    tau = [L*k*(-wm(1)^2-wm(2)^2+wm(3)^2+wm(4)^2);
        L*k*(wm(1)^2-wm(2)^2-wm(3)^2+wm(4)^2);
        b*(-wm(1)^2+wm(2)^2-wm(3)^2+wm(4)^2)];
    omegadot = inv(I)*(tau -cross(omega,I*omega));
end

function thetadot = omega_to_thetadot(omega,theta)
    theta_rad = theta;
    T = [1 sin(theta_rad(1))*tan(theta_rad(2)) cos(theta_rad(1))*tan(theta_rad(2));
        0 cos(theta_rad(1)) -sin(theta_rad(1));
        0 sin(theta_rad(1))/cos(theta_rad(2)) cos(theta_rad(1))/cos(theta_rad(2))];
    thetadot = T*omega;
end

function v_out = ZerotoOne(v_in)
    for i = 1:length(v_in)
        if v_in(i)<0
            v_in(i) = 0;
        elseif v_in(i)>1
            v_in(i) = 1;
        end
    end
    v_out = v_in;
end