function measurement = generateMeasurements(time, onGround, xHistory, tick)
    % Measurements: column vector with Gyro, IMU and position measurement
    global state acc RAD_TO_DEG
    global dt g
    global t_latenz
    % 1)  IMU Measurements
    % define std for meas nois = [noise_gyro, noise_acc]
    % different if quadcopter is onGround
    measNoise_std_onGround = [0.0601, 0.0544, 0.0556, ...
        0.0008, 0.0007, 0.0011];
    measNoise_std_flying = [1, 1, 1, ...
        0.02, 0.02, 0.02];
    % calculate transformation matrix T_BE
    q0 = state(7);
    q1 = state(8);
    q2 = state(9);
    q3 = state(10); 
    T_BE = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2+q0*q3), 2*(q1*q3-q0*q2);...
        2*(q1*q2-q0*q3), q0^2-q1^2+q2^2-q3^2, 2*(q2*q3+q0*q1);...
        2*(q1*q3+q0*q2), 2*(q2*q3-q0*q1), q0^2-q1^2-q2^2+q3^2];
    % generate random noise        
    noise_IMU = nan(6,1);
    if (onGround)
        for i = 1:6
            noise_IMU(i) = normrnd(0,measNoise_std_onGround(i));
        end
    else
        for i = 1:6
            noise_IMU(i) = normrnd(0,measNoise_std_flying(i));
        end
    end
    % calculate measurements and add noise
    measurement = nan(9,1);
    measurement(1:3) = RAD_TO_DEG*state(11:13)+noise_IMU(1:3);
    measurement(4:6) = 1/g*T_BE*(acc-[0;0;-g])+noise_IMU(4:6);
    
    % 2.) position Measurements
    measNoise_pos = [0.002, 0.001, 0.001];
    dt_pos = 0.1;
    k_latenz = t_latenz/dt;
    if (mod(time,dt_pos) == 0) && (tick-k_latenz>0)
        % only at 10 Hz
        for i = 1:3
            % use position state at tick-k_latenz
            measurement(6+i) = xHistory(i, tick-k_latenz)+normrnd(0, measNoise_pos(i));
        end
    end 
end