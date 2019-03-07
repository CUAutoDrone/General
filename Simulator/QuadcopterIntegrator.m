function [t_out,w_out,q_out,p_out,v_out] = QuadcopterIntegrator(q,w,p,v,tau_in,thrust,tspan,numsteps,I,m,g)

%integrator properties
newton_tol = 10^-7;

tspace = linspace(tspan(1),tspan(2),numsteps);
qspace = zeros(length(tspace),4);
wspace = zeros(3,length(tspace));
pspace = zeros(3,length(tspace));
vspace = zeros(3,length(tspace));

h = (tspan(2)-tspan(1))/numsteps;
step=1;
for t = tspace
    qspace(step,:) = q;
    wspace(:,step) = w;
    pspace(:,step) = p;
    vspace(:,step) = v;
    
    pk = I*w;
    phi_init = h/2*w;
    phi_out = newton_phi(phi_init,pk,tau_in,I,h,newton_tol);
    fk = [sqrt(1-phi_out'*phi_out);phi_out];
    q1 = quatmultiply(q,fk');
    w1 = inv(I)*(2/h*sqrt(1-phi_out'*phi_out)*I*phi_out+cross(phi_out,I*phi_out)+h*tau_in);
    
    t_rot = quatmultiply(quatinv(q),quatmultiply([0,0,0,thrust],q));
    f_in = t_rot(2:4)'-[0;0;m*g];
    mom_k = m*v;
    p1 = p+mom_k/m*h+f_in*h^2/(2*m);
    mom_k1 = m*(p1-p)/h+h/2*f_in;
    v1 = mom_k1/m; 
    
    q = q1;
    w = w1;
    p = p1;
    v = v1;
    
    step = step+1;
end

w_out = wspace;
q_out = qspace;
p_out = pspace;
v_out = vspace;
t_out = tspace;

end

function phi_out = newton_phi(phi_init,pk,tau_in,I,h,tol)
stepcount = 0;
phi = phi_init;
%fval = sqrt(1-phi'*phi)*I*phi-cross(phi,I*phi);
fval = [1;1;1];
while fval > tol
   fval = sqrt(1-phi'*phi)*I*phi+cross(phi,I*phi)-pk*h/2+tau_in*h^2/2;
   fprime = 2/h*(sqrt(1-phi'*phi)*I-I*(phi*phi')/sqrt(1-phi'*phi)+crs(phi)*I-crs(I*phi));
   phi = phi-inv(fprime)*fval;
   stepcount = stepcount+1;
   if stepcount>10
       disp('Error: Newton Failed to Converge')
   end
end
phi_out = phi;

end

function crs_out = crs(v)
crs_out = [0 -v(3) v(2);
           v(3) 0 -v(1);
           -v(2) v(1) 0];
end