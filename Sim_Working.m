clear all;

%Simulation Parameters
%Global simulation start and stop times
start_time = 0; %sec
end_time = 5; %sec
%This is the sensor dt - corresponds to the control loop refresh rate
dt = 0.005; %sec



%helper quantities
times = start_time:dt:end_time;
N = numel(times);
step = 0; %timestep counter
sum = [0;0;0]; %integral summer

%Drone Physical Properties
%Moment of Inertia Tensor 
I = [1 0 0; 0 1 0; 0 0 1];
%Assuming square quadrotor, distance between motor and axis
L = 1;
%propellor drag coefficient
b = 1;
%propellor torque coefficient
k = 100;

%Controller properties
Kp = 3;
Ki = .02;
Kd = .4;

%initial conditions
x = [0; 0; 10]; %initial position, m
xdot = zeros(3,1); %initial velocity, m/s
theta = [deg2rad(20);0;0]; %initial angular position, in 3-2-1 euler
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

%Vis Stuff
Copter_Model = [1 -1 -1 1;
                1 -1 1 -1;
                0 0  0 0];
        
Copter_Model_TH = euler_rotate(Copter_Model,theta);


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
    
    states = [states rad2deg(theta)];
    Copter_Model_TH = cat(3,Copter_Model_TH,euler_rotate(Copter_Model,theta));
    %disp(rad2deg(theta))
    step = step+1;
       
end
figure
plot(times,(states(1,2:end)))

% figure;
% view(20,20)
% zlim manual;
% axis([-2 2 -2 2 -2 2])
% for j = 1:100
%     for i = 1:length(Copter_Model_TH)
%         %plot3(Copter_Model_TH(1,:,i),Copter_Model_TH(2,:,i),Copter_Model_TH(3,:,i),'b')
%         plot3(Copter_Model_TH(1,1:2,i),Copter_Model_TH(2,1:2,i),Copter_Model_TH(3,1:2,i),'b')
%         hold on
%         plot3(Copter_Model_TH(1,3:4,i),Copter_Model_TH(2,3:4,i),Copter_Model_TH(3,3:4,i),'b')
%         scatter3(Copter_Model_TH(1,:,i),Copter_Model_TH(2,:,i),Copter_Model_TH(3,:,i),[200,200,200,200],'r','filled')
%         hold off
%         set(findall(gca, 'Type', 'Line'),'LineWidth',5);
%         axis([-2 2 -2 2 -2 2])
%         drawnow
%         pause(0.01)
%     end
% end


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

function v_body = euler_rotate(v_inert,theta)
    roll = theta(1);
    pitch = theta(2);
    yaw = theta(3);
    Rz = [cos(yaw) -sin(yaw) 0;
        sin(yaw) cos(yaw) 0;
        0 0 1];
    Ry = [cos(yaw) 0 -sin(pitch);
        0 1 0;
        sin(pitch) 0 cos(pitch)];
    Rx = [1 0 0;
        0 cos(roll) sin(roll);
        0 -sin(roll) cos(roll)];
    v_body = Rx*Ry*Rz*v_inert;
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