function predictionStep_EKF(motorCmds, timestep)
    global mB Ixx Iyy Izz kappa l_arm
    global g
    global motTimeConst_EKF prevMotorForces_EKF
    global xm Pm xp Pp
    global Q
    
    n_state = 16;
    n_v = 9;

    s1 = xm(1);
    s2 = xm(2);
    s3 = xm(3);
    v1 = xm(4);
    v2 = xm(5);
    v3 = xm(6);
    q0 = xm(7);
    q1 = xm(8);
    q2 = xm(9);
    q3 = xm(10);
    p = xm(11);
    q = xm(12);
    r = xm(13);
    nb1 = xm(14);
    nb2 = xm(15);
    nb3 = xm(16);
    
    % 1.) motTimeConst: currently at zero, so won't do much
    motorForces_des = 2.13e-11.*motorCmds.^2+1.03e-6*motorCmds+5.48e-4;
    if motTimeConst_EKF== 0
        c = 0;
    else
        c = exp(-timestep/motTimeConst_EKF);
    end
    cp1 = c*prevMotorForces_EKF(1)+(1-c)*motorForces_des(1);
    cp2 = c*prevMotorForces_EKF(2)+(1-c)*motorForces_des(2);
    cp3 = c*prevMotorForces_EKF(3)+(1-c)*motorForces_des(3);
    cp4 = c*prevMotorForces_EKF(4)+(1-c)*motorForces_des(4);       
    prevMotorForces_EKF(1) = cp1;
    prevMotorForces_EKF(2) = cp2;
    prevMotorForces_EKF(3) = cp3;
    prevMotorForces_EKF(4) = cp4;
    
    % 2.) calculate force moments
    f_tot = cp1+cp2+cp3+cp4;
    n1 = -l_arm*cp1-l_arm*cp2+l_arm*cp3+l_arm*cp4;
    n2 = -l_arm*cp1+l_arm*cp2+l_arm*cp3-l_arm*cp4;
    n3 = -kappa*cp1+kappa*cp2-kappa*cp3+kappa*cp4;   
    thrust_norm = (f_tot)/mB;
    
    if(motorCmds==0)
        % quadcopter still on ground
        xp(1) = s1;
        xp(2) = s2;
        xp(3) = s3;
        xp(4) = v1;
        xp(5) = v2;
        xp(6) = v3;
        xp(7) = q0;
        xp(8) = q1;
        xp(9) = q2;
        xp(10) = q3;
        xp(11) = p;
        xp(12) = q;
        xp(13) = r;
        xp(14) = nb1;
        xp(15) = nb2;
        xp(16) = nb3;    
        for i = 1:n_state
            for j = 1:n_state
                Pp(i,j) = Pm(i,j);
            end
        end
    else
        % quadcopter is flying
        % State prediction
        xp(1) = s1+timestep*v1;
        xp(2) = s2+timestep*v2;
        xp(3) = s3+timestep*v3;
        xp(4) = v1+timestep*(2*(q1*q3+q0*q2)*thrust_norm);
        xp(5) = v2+timestep*(2*(q2*q3-q0*q1)*thrust_norm);
        xp(6) = v3+timestep*((q0^2-q1^2-q2^2+q3^2)*thrust_norm-g);
        xp(7) = q0+timestep*0.5*(-p*q1-q*q2-r*q3);
        xp(8) = q1+timestep*0.5*(p*q0+r*q2-q*q3);
        xp(9) = q2+timestep*0.5*(q*q0-r*q1+p*q3);
        xp(10) = q3+timestep*0.5*(r*q0+q*q1-p*q2);
        xp(11) = p+timestep*(n1-nb1-(Izz-Iyy)*q*r)/Ixx;
        xp(12) = q+timestep*(n2-nb2-(Ixx-Izz)*r*p)/Iyy;
        xp(13) = r+timestep*(n3-nb3-(Iyy-Ixx)*p*q)/Izz;
        xp(14) = nb1;
        xp(15) = nb2;
        xp(16) = nb3;
        
        % Variance Prediction
        % 1. Build A matrix
        A_EKF = zeros(n_state, n_state);
        for i = 1:n_state
            A_EKF(i,i) = single(1);
        end

        A_EKF(1,4) = timestep;
        A_EKF(2,5) = timestep;
        A_EKF(3,6) = timestep;

        A_EKF(4,7) = timestep*2*q2*thrust_norm;
        A_EKF(4,8) = timestep*2*q3*thrust_norm;
        A_EKF(4,9) = timestep*2*q0*thrust_norm;
        A_EKF(4,10) = timestep*2*q1*thrust_norm;
        A_EKF(5,7) = -timestep*2*q1*thrust_norm;
        A_EKF(5,8) = -timestep*2*q0*thrust_norm;
        A_EKF(5,9) = timestep*2*q3*thrust_norm;
        A_EKF(5,10) = timestep*2*q2*thrust_norm;
        A_EKF(6,7) = timestep*2*q0*thrust_norm;
        A_EKF(6,8) = -timestep*2*q1*thrust_norm;
        A_EKF(6,9) = -timestep*2*q2*thrust_norm;
        A_EKF(6,10) = timestep*2*q3*thrust_norm;

        A_EKF(7,8) = -timestep*0.5*p;
        A_EKF(7,9) = -timestep*0.5*q;
        A_EKF(7,10) = -timestep*0.5*r;
        A_EKF(7,11) = -timestep*0.5*q1;
        A_EKF(7,12) = -timestep*0.5*q2;
        A_EKF(7,13) = -timestep*0.5*q3;
        A_EKF(8,7) = timestep*0.5*p;
        A_EKF(8,9) = timestep*0.5*r;
        A_EKF(8,10) = -timestep*0.5*q;
        A_EKF(8,11) = timestep*0.5*q0;
        A_EKF(8,12) = -timestep*0.5*q3;
        A_EKF(8,13) = timestep*0.5*q2;
        A_EKF(9,7) = timestep*0.5*q;
        A_EKF(9,8) = -timestep*0.5*r;
        A_EKF(9,10) = timestep*0.5*p;
        A_EKF(9,11) = timestep*0.5*q3;
        A_EKF(9,12) = timestep*0.5*q0;
        A_EKF(9,13) = -timestep*0.5*q1;
        A_EKF(10,7) = timestep*0.5*r;
        A_EKF(10,8) = timestep*0.5*q;
        A_EKF(10,9) = -timestep*0.5*p;
        A_EKF(10,11) = -timestep*0.5*q2;
        A_EKF(10,12) = timestep*0.5*q1;
        A_EKF(10,13) = timestep*0.5*q0;

        A_EKF(11,12) = timestep*r*(Iyy-Izz)/Ixx; 
        A_EKF(11,13) = timestep*q*(Iyy-Izz)/Ixx; 
        A_EKF(11,14) = -timestep/Ixx; 
        A_EKF(12,11) = timestep*r*(Izz-Ixx)/Iyy; 
        A_EKF(12,13) = timestep*p*(Izz-Ixx)/Iyy; 
        A_EKF(12,15) = -timestep/Iyy; 
        A_EKF(13,11) = timestep*q*(Ixx-Iyy)/Izz; 
        A_EKF(13,12) = timestep*p*(Ixx-Iyy)/Izz; 
        A_EKF(13,16) = -timestep/Izz; 
        
        A_EKF = single(A_EKF);
        %2. build L matrix
        L_EKF = zeros(n_state,n_v);
        L_EKF(4,1) = single(1);
        L_EKF(5,2) = single(1);
        L_EKF(6,3) = single(1);
        L_EKF(11,4) = single(1);
        L_EKF(12,5) = single(1);
        L_EKF(13,6) = single(1);
        L_EKF(14,7) = single(1);
        L_EKF(15,8) = single(1);
        L_EKF(16,9) = single(1);
                
        % 3. Calculate Pp
        Pp = A_EKF*Pm*A_EKF'+L_EKF*Q*L_EKF';
    end
    
    xp = normalizeQuaternions(xp);
    Pp = cleanVariance(Pp);
end