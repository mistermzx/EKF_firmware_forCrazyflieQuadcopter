function updateWithIMU_EKF (acc, gyro)
    global xp Pp 
    global g DEG_TO_RAD 
    global xm_IEKF Pm_IEKF
    global R_IMU


    n_state = length(xp); 
    
    q0 = xm_IEKF(7);
    q1 = xm_IEKF(8);
    q2 = xm_IEKF(9);
    q3 = xm_IEKF(10);
    p = xm_IEKF(11);
    q = xm_IEKF(12);
    r = xm_IEKF(13);
    
   % 1.) build H_IMU matrix
   H_IMU = zeros(6, n_state);
   H_IMU(1,11) = 1;
   H_IMU(2,12) = 1;
   H_IMU(3,13) = 1;
   
   H_IMU(4,7) = -g*2*q2;
   H_IMU(4,8) = g*2*q3;
   H_IMU(4,9) = -g*2*q0;
   H_IMU(4,10) = g*2*q1;
   H_IMU(5,7) = g*2*q1;
   H_IMU(5,8) = g*2*q0;
   H_IMU(5,9) = g*2*q3;
   H_IMU(5,10) = g*2*q2; 
   H_IMU(6,7) = g*2*q0;
   H_IMU(6,8) = -g*2*q1;
   H_IMU(6,9) = -g*2*q2;
   H_IMU(6,10) = g*2*q3;   
    
    % 2.) calculate gain matrix
    K_IMU = Pp*H_IMU'*(H_IMU*Pp*H_IMU'+R_IMU)^-1;
   
   %3.) define innovation
   innovation_IMU = zeros(6,1);
   innovation_IMU(1) = gyro(1)-p;
   innovation_IMU(2) = gyro(2)-q;
   innovation_IMU(3) = gyro(3)-r;
   innovation_IMU(4) = acc(1)-g*2*(q1*q3-q0*q2);
   innovation_IMU(5) = acc(2)-g*2*(q2*q3+q0*q1);
   innovation_IMU(6) = acc(3)-g*(q0^2-q1^2-q2^2+q3^2);
   innovation_IMU = innovation_IMU-H_IMU*(xp-xm_IEKF);
   
   %4.) calculate posterior mean
   xm_IEKF = xp + K_IMU*innovation_IMU;
   xm_IEKF = normalizeQuaternions(xm_IEKF);
   
   %5.) calculate posterior variance
   Pm_IEKF = (eye(n_state)-K_IMU*H_IMU)*Pp;
   Pm_IEKF = cleanVariance(Pm_IEKF);
end