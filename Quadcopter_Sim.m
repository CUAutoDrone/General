clearvars;

%Simulation Parameters
%Global simulation start and stop times
start_time = 0; %sec
end_time = 2; %sec
%This is the sensor dt - corresponds to the control loop refresh rate
dt = 0.005; %sec
g = 9.806;


%helper quantities
times = start_time:dt:end_time;
N = numel(times);
step = 1; %timestep counter
sum = [0;0;0]; %integral summer

%Drone Physical Properties
%Moment of Inertia Tensor, kg m^2
I = [6.77e-2 1.617e-5 914e-9; 
    1.617e-5 1.129e-1 -3.923e-9; 
    914e-9 -3.923e-9 5.546e-2];
%Assuming square quadrotor, distance between motor and axis
R = 0.313;
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
%quadcopter mass
m = 2.568; %kg
rcm = [.02577e-3;-11.281e-3;-3.49e-9];

%Controller properties
Kp = [20;20;50];
Ki = [0;0;0];
Kd = [0.58;0.58;2];

%initial conditions
x = [0; 0; 10]; %initial position, m
xdot = zeros(3,1); %initial velocity, m/s
theta = [0;deg2rad(-10);0]; %initial angular position, in 3-2-1 euler
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

%Plot Parameters
states = zeros(6, N);

%Vis Stuff
Copter_Model = [1 -1 -1 1;
                1 -1 1 -1;
                0 0  0 0];
        
Copter_Model_TH = euler_rotate(Copter_Model,theta);


%step through sim
for time = start_time:dt:end_time
    err = theta_r-theta;
    sum = sum+err;
    
    if step == 1
        u = Kp.*err+Ki.*sum*dt;
    else
        u = Kp.*err+Ki.*sum*dt+Kd.*(err-prev_err)/dt;
    end
    prev_err = err;
    
    ctrl = [throttle;
            u(1);
            u(2);
            u(3)];
    uc = ctrl_to_uc*ctrl;
    wm = w_max*ZerotoOne(uc);
    %wm = [600;600;600;600];
    
    state = [theta(1);theta(2);theta(3);omega(1);omega(2);omega(3);x(1);x(2);x(3);xdot(1);xdot(2);xdot(3)];
    [t_out,y] = ode45(@(t,y) propagator(t,y,wm,I,L,b,k,rcm,m,g),[time,time+dt],state);
    theta(1) = y(end,1);
    theta(2) = y(end,2);
    theta(3) = y(end,3);
    omega(1) = y(end,4);
    omega(2) = y(end,5);
    omega(3) = y(end,6);
    x(1:3) = y(end,7:9);
    xdot(1:3) = y(end,10:12);
    
    states(1:3, step) = rad2deg(theta);
    states(4:6, step) = x;
    Copter_Model_TH = cat(3,Copter_Model_TH,euler_rotate(Copter_Model,theta));
    %disp(rad2deg(theta))
    step = step+1;
       
end

figure;
plot(times,states(6,:))

% hold on;
% roll = plot(times,(states(1,:)));
% pitch = plot(times,(states(2,:)));
% yaw = plot(times,(states(3,:)));
% legend([roll, pitch, yaw], ["roll err", "pitch err", "yaw err"]);
% hold off;

%3D Animation
% figure;
% view(20,20)
% zlim manual;
% axis([-2 2 -2 2 -2 2])
% for j = 1:100
%     for i = 1:length(Copter_Model_TH)
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


function state_dot = propagator(t,state,wm,I,L,b,k,rcm,m,g)
    omega = [state(4);state(5);state(6)];
    theta = [state(1);state(2);state(3)];
    thetadot = omega_to_thetadot(omega,theta);
    omegadot = angular_acceleration(wm,omega, theta, I,L,b,k,rcm,m,g);
    xdoubledot = acceleration(theta,m,g,wm,k);
    state_dot = [thetadot(1);
                thetadot(2);
                thetadot(3);
                omegadot(1);
                omegadot(2);
                omegadot(3)
                state(10);
                state(11);
                state(12);
                xdoubledot(1);
                xdoubledot(2);
                xdoubledot(3)];
                
end

function xdoubledot = acceleration(theta,m,g,wm,k)
    thrust = k*(wm(1)^2+wm(2)^2+wm(3)^2+wm(4)^2);
    thrust_vect = thrust*[-sin(theta(1))*sin(theta(3))-cos(theta(1))*cos(theta(3))*sin(theta(2));
                          cos(theta(1))*sin(theta(3))*sin(theta(2))-cos(theta(3))*sin(theta(3));
                          -cos(theta(1))*cos(theta(2))];
    xdoubledot = [thrust_vect(1)/m;
                  thrust_vect(2)/m;
                  thrust_vect(3)/m+g/m];

end

function omegadot = angular_acceleration(wm, omega, theta, I, L, b, k,rcm,m,g)
    rcm_crs = [0 -rcm(3) rcm(2);
                rcm(3) 0 -rcm(1);
                -rcm(2) rcm(1) 0];
    tau_grav = rcm_crs*m*[-g*sin(theta(2));
                    -cos(theta(2))*sin(theta(1));
                    cos(theta(1))*cos(theta(2))];
    tau_motors = [L*k*(-wm(1)^2-wm(2)^2+wm(3)^2+wm(4)^2);
        L*k*(wm(1)^2-wm(2)^2-wm(3)^2+wm(4)^2);
        b*(-wm(1)^2+wm(2)^2-wm(3)^2+wm(4)^2)];
    tau = tau_motors+tau_grav;
    %disp(tau)
    omegadot = I\(tau -cross(omega,I*omega));
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
