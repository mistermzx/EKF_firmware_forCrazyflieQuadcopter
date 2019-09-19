function stateEstimator_Update(motorCmds, measurements, tick)
    global state_estimate
    global lastUpdate lastPredictionStep
    global dt_mainLoop
    global xm Pm xp Pp
    global motorCmdsAccumulator gyroAccumulator accAccumulator
    global motorCmds_count gyro_count acc_count
    global g DEG_TO_RAD 
    
    global xm_IEKF Pm_IEKF n_IEKF
    
    doneUpdate = false;
    Update_Prediction = false;
    
    % add measurements and motor cmds to accumulator
    gyroAccumulator = gyroAccumulator+DEG_TO_RAD*measurements(1:3);
    gyro_count = gyro_count+1;
    accAccumulator = accAccumulator+g*measurements(4:6);
    acc_count = acc_count+1;
    motorCmdsAccumulator = motorCmdsAccumulator+motorCmds;
    motorCmds_count = motorCmds_count+1;   
    
    if (tick-lastPredictionStep)>=10
        % do at 100 Hz
        % 1. Average IMU meas and motor Cmds
        gyroAccumulator = gyroAccumulator/gyro_count;
        accAccumulator = accAccumulator/acc_count;
        motorCmdsAccumulator = motorCmdsAccumulator/motorCmds_count;
        % 2. Perform prediction step
        dt_prediction = (tick-lastUpdate)*dt_mainLoop;
        predictionStep_EKF(motorCmdsAccumulator, dt_prediction);
        
        % 3. Do iterated IMU update
        % intialize IEKF
        xm_IEKF = xp;
        Pm_IEKF = Pp;
        % iteration
        for i = 1:n_IEKF
            updateWithIMU_EKF(accAccumulator, gyroAccumulator);
        end
        % set estimate 
        xm = xm_IEKF;
        Pm = Pm_IEKF;
        % reset accumulator and counter
        accAccumulator = zeros(3,1);
        gyroAccumulator = zeros(3,1);
        acc_count = 0;
        gyro_count = 0;
        motorCmds_count = 0;
        motorCmdsAccumulator = zeros(4,1);
        % reset counter
        lastPredictionStep = tick;
        lastUpdate = tick;
        doneUpdate = true;     
        Update_Prediction = true;
    end
    
    % Do posiiton update if meas is available
    posMeas = measurements(7:9);
    if (~isnan(posMeas))
        updateWithPosition_EKF(posMeas);
        lastUpdate = tick;
        doneUpdate = true;
    end
    
    % save xm into state estimate
    if doneUpdate
        state_estimate = xm;
    end
    % modification in angular velocity: save gyroMeas if no update made
    if ~Update_Prediction
        state_estimate(11:13) = DEG_TO_RAD*measurements(1:3);
    end
end