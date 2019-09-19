function simulateDynamics (input)
global mB Ixx Iyy Izz kappa l_arm nb1 nb2 nb3 
global state dt acc
global g

% random disturbances in forces and moments:
f_x = normrnd(0, 2e-3);
f_y = normrnd(0, 2e-3);
f_z = normrnd(0, 2e-3);
n_x = normrnd(0, 3e-4); 
n_y = normrnd(0, 3e-4); 
n_z = normrnd(0, 3e-4); 

% state
s1 = state(1);
s2 = state(2);
s3 = state(3);
v1 = state(4);
v2 = state(5);
v3 = state(6);
q0 = state(7);
q1 = state(8);
q2 = state(9);
q3 = state(10);
p = state(11);
q = state(12);
r = state(13);
% input
f_tot = input(1);
thrust_norm = f_tot/mB;
n1 = input(2);
n2 = input(3);
n3 = input(4);
% calculate translational acceleration for accMeas
acc(1) = 2*(q1*q3+q0*q2)*thrust_norm;
acc(2) = 2*(q2*q3-q0*q1)*thrust_norm;
acc(3) = (q0^2-q1^2-q2^2+q3^2)*thrust_norm-g;
% perturbed moment of inertia
Ixx_pert = Ixx*1.1;
Iyy_pert = Iyy*1.1;
Izz_pert = Izz*1.1;
% simulate system using euler one step
state(1) = s1+dt*v1;
state(2) = s2+dt*v2;
state(3) = s3+dt*v3;
state(4) = v1+dt*(2*(q1*q3+q0*q2)*thrust_norm+f_x/mB);
state(5) = v2+dt*(2*(q2*q3-q0*q1)*thrust_norm+f_y/mB);
state(6) = v3+dt*((q0^2-q1^2-q2^2+q3^2)*thrust_norm-g+f_z/mB);
state(7) = q0+dt*0.5*(-p*q1-q*q2-r*q3);
state(8) = q1+dt*0.5*(p*q0+r*q2-q*q3);
state(9) = q2+dt*0.5*(q*q0-r*q1+p*q3);
state(10) = q3+dt*0.5*(r*q0+q*q1-p*q2);
state(11) = p+dt*(n1-nb1-(Izz_pert-Iyy_pert)*q*r+n_x)/Ixx_pert;
state(12) = q+dt*(n2-nb2-(Ixx_pert-Izz_pert)*r*p+n_y)/Iyy_pert;
state(13) = r+dt*(n3-nb3-(Ixx_pert-Ixx_pert)*p*q+n_z)/Izz_pert;
% normalize quaternions
state(7:10) = state(7:10)./norm(state(7:10));
end