function forceMoments = calcForceMomentsFromMotorCmds(motorCmds)
    global kappa l_arm
    global prevMotorForces_simulation motTimeConst_simulation
    global dt

    % calculate motorForces including motTimeConst
    motorForces_des = 2.13e-11.*motorCmds.^2+1.03e-6*motorCmds+5.48e-4;
    if (motTimeConst_simulation == 0)
        c = 0;
    else 
        c = exp(-dt/motTimeConst_simulation);
    end  
    motorForces = c*prevMotorForces_simulation+(1-c)*motorForces_des;
    prevMotorForces_simulation = motorForces;
    
    % calculate forceMoments through mixer matrix: with model perturbations
    Mixer = [1 1 1 1;...
        -l_arm -l_arm +l_arm +1.05*l_arm;...
        -l_arm +l_arm +l_arm -1.05*l_arm;...
        -kappa +kappa -0.9*kappa +kappa];
    forceMoments = Mixer*motorForces;
end