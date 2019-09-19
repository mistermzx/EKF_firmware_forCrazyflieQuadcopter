function updateWithPosition_EKF(posMeas)
    global xm Pm
    global R_pos
    global xm_old xm_old2
    global latencyCompensation
    
    n_state = length(xm);
    
    Pm_calc = Pm;
    
    if latencyCompensation
        xm_calc = xm_old2;
    else 
        xm_calc = xm;
    end
    
    % 1.) build H_pos matrix
    H_pos = zeros(3, n_state);
    H_pos(1,1) = 1;
    H_pos(2,2) = 1;
    H_pos(3,3) = 1; 
    
    %2.) calculate K_pos
    K_pos = Pm*H_pos'*(H_pos*Pm*H_pos'+R_pos)^-1;
    
    %3.) define innovation
    innovation_pos = zeros(3,1);
    innovation_pos(1) = posMeas(1)-xm_calc(1);
    innovation_pos(2) = posMeas(2)-xm_calc(2);
    innovation_pos(3) = posMeas(3)-xm_calc(3);
    
    %4.) calculate posterior mean
    tmp3 = K_pos*innovation_pos;
    for i = 1:6
        xm(i) = xm(i) + tmp3(i);
    end
    xm = normalizeQuaternions(xm);
    
    % for latency 
    xm_old2 = xm_old;
    xm_old = xm;
    
    %5.) calculate posterior variance
    Pm = (eye(n_state)-K_pos*H_pos)*Pm;
end