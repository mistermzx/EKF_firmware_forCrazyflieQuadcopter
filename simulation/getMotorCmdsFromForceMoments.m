function motorCmds = getMotorCmdsFromForceMoments(forceMoments)
    global kappa l_arm

    % invert mixer matrix to calculate motor forces
    Mixer = [1 1 1 1;...
        -l_arm -l_arm +l_arm +l_arm;...
        -l_arm +l_arm +l_arm -l_arm;...
        -kappa +kappa -kappa +kappa];
    motorForces = inv(Mixer)*forceMoments;
    
    % calculate motorCmds by solving quadratic equation and saturating
    motorCmds = nan(4,1);
    a = 2.13e-11;
    b = 1.03e-6;
    c = 5.48e-4;
    for i = 1:4
        if (motorForces(i)<0)
            motorForces(i) = 0;
        end
        motorCmds(i) = -b/(2*a)+sqrt(b^2/(4*a^2)-(c-motorForces(i))/a);
        motorCmds(i) = round(motorCmds(i));
        if (motorCmds(i)>65535)
            motorCmds(i) = 65535;
        end
        if motorCmds(i)<0
            motorCmds(i) = 0;
        end
    end
end