function input = getControllerOutput_PID (state_estimate, des_pos, des_yaw)
global g
global mB Ixx Iyy Izz 
global PID_posx PID_posy PID_velx PID_vely 
global PID_roll PID_pitch PID_yaw PID_p PID_q PID_r
global dt

% define control parameter: Try and error
Kp_pos_xy = 1/(3*0.5); 
Kp_vel_xy = 1/(3*0.25);
natFrequency_z = 2;
dampingRatio_z = 0.7;
Kp_rollpitch = 1/0.12;
Kp_yaw = 1/0.2;
Kp_pq = 1/0.04;
Kp_r = 1/0.1;
% feed forward term of the control algorithm (bias moments)
n_bias = [0.8e-3, 1.4e-3, -0.2e-3];
% use current stateEstimate: transform attitude to Euler angles
s1 = state_estimate(1);
s2 = state_estimate(2);
s3 = state_estimate(3);
v1 = state_estimate(4);
v2 = state_estimate(5);
v3 = state_estimate(6);
q = state_estimate(7:10);
RPY = quaternion2Euler(q);
roll = RPY(1);
pitch = RPY(2);
yaw = RPY(3);
p = state_estimate(11);
q = state_estimate(12);
r = state_estimate(13);

% Cascaded Controller: using saturations on position and velocity control
% 1.) Horizontal Position + Yaw
PID_posx = PID_posx.calculatePIDOutput((des_pos(1)-s1), Kp_pos_xy,Kp_pos_xy/60,3*Kp_pos_xy, dt);
PID_posx = PID_posx.constrainWithAntiWindup(PID_posx.output, 1.1, dt);
des_vel_x = PID_posx.output;
PID_posy = PID_posy.calculatePIDOutput((-s2+des_pos(2)), Kp_pos_xy, Kp_pos_xy/60, Kp_pos_xy,dt);
PID_posy = PID_posy.constrainWithAntiWindup(PID_posy.output, 1.1, dt);
des_vel_y = PID_posy.output;

PID_velx = PID_velx.calculatePIDOutput((-v1+des_vel_x), Kp_vel_xy,0, 0, dt);
PID_velx = PID_velx.constrainWithAntiWindup(PID_velx.output, 30*pi/180, dt);
des_acc_x = PID_velx.output;
PID_vely = PID_vely.calculatePIDOutput((-v2+des_vel_y), Kp_vel_xy, 0, 0, dt);
PID_vely = PID_vely.constrainWithAntiWindup(PID_vely.output, 30*pi/180, dt);
des_acc_y = PID_vely.output;

des_roll = 1/g*(-des_acc_y*cos(yaw)+sin(yaw)*des_acc_x);
des_pitch = 1/g*(des_acc_x*cos(yaw)+sin(yaw)*des_acc_y);

PID_roll = PID_roll.calculatePIDOutput((-roll+des_roll),Kp_rollpitch, Kp_rollpitch/2, 0*Kp_rollpitch/60, dt);
des_p = PID_roll.output;
PID_pitch = PID_pitch.calculatePIDOutput((-pitch+des_pitch), Kp_rollpitch,Kp_rollpitch/2, 0*Kp_rollpitch/60, dt);
des_q = PID_pitch.output;
PID_yaw = PID_yaw.calculatePIDOutput((-yaw+des_yaw), Kp_yaw, Kp_yaw/6,7/120*Kp_yaw, dt);
des_r = PID_yaw.output;

PID_p = PID_p.calculatePIDOutput((-p+des_p), Kp_pq, 2*Kp_pq,0*Kp_pq/100, dt);
des_pRate = PID_p.output;
PID_q = PID_q.calculatePIDOutput((-q+des_q), Kp_pq, 2*Kp_pq,0*Kp_pq/100, dt);
des_qRate = PID_q.output;
PID_r = PID_r.calculatePIDOutput((-r+des_r), Kp_r, 167/1200*Kp_r,0, dt);
des_rRate = PID_r.output;

n1 = des_pRate*Ixx+n_bias(1);
n2 = des_qRate*Iyy+n_bias(2);
n3 = des_rRate*Izz+n_bias(3);

% 2. Vertical Position
des_acc_z = -2*dampingRatio_z*natFrequency_z*v3-natFrequency_z^2*(s3-des_pos(3));

normThrust = (g+des_acc_z)/(cos(roll)*cos(pitch));

f_tot = normThrust*mB;

% return
input = [f_tot;n1;n2;n3];
end