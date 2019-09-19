%% Main Simulation
% Authored by Martin Ziran Xu: simulation of a quadcopter system
% with a cascaded PID controller and an IEKF as state estimation
% written within the Bachelor Thesis: 
% Implementation of nonlinear model-based state estimation algorithms
% @ TU Berlin: Chair of measurement and control
% 10/31/2018
%% 
clear; clc; close all;
restoredefaultpath;
%% Should I plot the results?
plotting = false;
%% IEKF iterations
% for n_IEKF = 1-->EKF
global n_IEKF
n_IEKF = 1;
%% Enable latency compensation a la Prit?
% estimates diverge if enabled
global latencyCompensation
latencyCompensation = false;
%% Setpoint
des_pos = [0.7,-0.1,1];
des_yaw = 0;
n_state = 13;
%% Time
global dt
dt = 0.001; %timestep of simulation
tFinal = 15; 
tStart = 3; % no control before this time, quadcopter will just stay on ground
time = 0:dt:tFinal;
n = length(time);
%% Latency of position measurement
global t_latenz 
t_latenz = 0.2;
%% Motor Time Constants
% our simulated quadcopter has a motTimeConst of 0.01, but the EKF does
% not know about it and assumes zero time constant
global motTimeConst_simulation motTimeConst_EKF prevMotorForces_simulation prevMotorForces_EKF
motTimeConst_simulation = 0.01;
motTimeConst_EKF = 0;
prevMotorForces_simulation = zeros(4,1);
prevMotorForces_EKF = zeros(4,1);
%% constants
global g RAD_TO_DEG DEG_TO_RAD
g = 9.81;
RAD_TO_DEG = 180/pi;
DEG_TO_RAD = pi/180;
%% Crazyflie parameters
global mB Ixx Iyy Izz kappa l_arm nb1 nb2 nb3 
mB = 32.64e-3;
Ixx = 26.3052e-6;
Iyy = 26.0375e-6;
Izz = 30.4167e-6;
kappa = 0.005964552;
l_arm = 32.5e-3;
%% Bias Moments
% To be added to the simulated quasdcopter
nb1 = 0.5215e-3;
nb2 = 1.3797e-3;
nb3 = -0.1960e-3;
%% Bookeeping
xHistory = nan(n_state, n);
inputHistory = nan(4, n);
measurementHistory = nan(9, n);
estimateHistory = nan(n_state+3,n);
%% Initialize state of simulation
global state acc
q = [0.9983, 0.0044, -0.0035, 4.0908e-6];
pos = [1.0105, -0.05445, 0.4279];
state = [pos,0,0,0,q, 0, 0,0]';
acc = [0, 0, 0]'; % translational acceleration for accMeas
xHistory(:,1) = state;
%% EKF Variables
global lastUpdate lastPredictionStep
global dt_mainLoop
global xm Pm xp Pp
global motorCmdsAccumulator gyroAccumulator accAccumulator
global motorCmds_count gyro_count acc_count
% set counter and timestep
lastUpdate = 0;
lastPredictionStep = 0;
dt_mainLoop = dt;
% EKF initializitation of state and variance
xm =[0.92, -0.08,0.45,0,0,0,1,0,0,0,0,0,0,0,0,0]';
xp = xm;
Pm = zeros(16,16);
initialVariance = [0.01, 0.01, 0.01, 0.000001, 0.000001,0.000001,...
    0.0001, 0.0001, 0.0001, 0.0001, 0.000001, 0.000001, 0.000001,...
    2.9004e-6, 7.8884e-6, 1.6350e-8];
for i = 1:16
    Pm(i,i) = initialVariance(i);
end
Pp = Pm;
% initialize accumulators and counter with zero
accAccumulator = zeros(3,1);
gyroAccumulator = zeros(3,1);
acc_count = 0;
gyro_count = 0;
motorCmds_count = 0;
motorCmdsAccumulator = zeros(4,1);
%% Latency Compensation
global xm_old xm_old2
xm_old = xm;
xm_old2 = xm;
%% Measurement and estimate
global state_estimate
state_estimate = xm;
%% Build controller objects
% Control parameters are in getControllerOutput_PID.m
global PID_posx PID_posy PID_posz PID_velx PID_vely PID_velz
global PID_roll PID_pitch PID_yaw PID_p PID_q PID_r
PID_posx = PIDController();
PID_posy = PIDController();
PID_posz = PIDController();
PID_velx = PIDController();
PID_vely = PIDController();
PID_velz = PIDController();
PID_roll = PIDController();
PID_pitch = PIDController();
PID_yaw = PIDController();
PID_p = PIDController();
PID_q = PIDController();
PID_r = PIDController();
%% Define model noise variance
global Q
Q = diag([5e-7,5e-7, 1e-4,...
            1e-2, 1e-2, 1e-4, ...
            1e-6, 1e-6,1e-8]);
%% Define meas noise variance :
global R_IMU R_pos
R_IMU = diag([DEG_TO_RAD*1.1424, DEG_TO_RAD*1.5955, DEG_TO_RAD*1.2572,...
        g*0.0283, g*0.0362, g*0.0208].^2);
R_pos = diag([0.0019, 0.0001, 0.0008].^2);
%% Simulation Loop
for i = 1:n-1
    if (time(i)<tStart)
        % nothing happens, motorCmds are Zero and the quadcopter does not
        % move
        motorCmds = zeros(4,1);
        inputHistory(:,i) = motorCmds;
        xHistory(:,i+1) = state;
        onGround = true;
    else
        % get controller output: returns forceMoments = [f_tot, nx, ny, nz]
        forceMoments = getControllerOutput_PID(state_estimate, des_pos, des_yaw);
        % calculates and saturates motorCmds 
        motorCmds = getMotorCmdsFromForceMoments(forceMoments);
        inputHistory(:,i) = motorCmds;
        % calculates forceMoments for quadcopter simulation: 
        % (including model perturbations)
        forceMoments = calcForceMomentsFromMotorCmds(motorCmds);
        % simulate state and saves the new one in state
        simulateDynamics(forceMoments);
        xHistory(:,i+1) = state;        
        onGround = false;
    end
        % generate measurements from state: [gyroMeas, accMeas, posMeas]
        measurements = generateMeasurements(time(i), onGround, xHistory, i);
        measurementHistory(:,i) = measurements;
        % calculate stateEstimate from motorCmds and measurements
        % (same as C implementation)
        stateEstimator_Update(motorCmds, measurements, i);
        estimateHistory(:,i+1) = state_estimate;
end
%% Save simulation data
save('IEKF_Data', 'xHistory', 'estimateHistory', 'inputHistory')
%% Calculate euler angles from Quaternions
eulerHistory_true = nan(3,n);
eulerHistory_estimate = nan(3,n);
for i = 1:n
    q = xHistory(7:10,i);
    eulerHistory_true(:,i) = RAD_TO_DEG*quaternion2Euler(q);
    q = estimateHistory(7:10,i);
    eulerHistory_estimate(:,i) = RAD_TO_DEG*quaternion2Euler(q);
end
%% Plotting stuff
if ~plotting
    error('You said no plotting');
end
%% x-Direction
fig = figure();
set(gcf, 'Position', [0, 0, 1000, 1000])
subplot(2,2,1)
hold on;
grid on;
plot(time, estimateHistory(1,:), '.', 'Markersize', 3);
plot(time, xHistory(1,:));
legend('estimate', 'truth');
legend('Location','southwest')
xlabel('Zeit in [s]')
ylabel('Position in [m]')
title('Position x')
subplot(2,2,2)
hold on;
grid on
plot(time, estimateHistory(4,:), '.', 'Markersize', 3)
plot(time, xHistory(4,:));
legend('estimate', 'truth');
legend('Location','southwest')
xlabel('Zeit in [s]')
ylabel('Geschwindigkeit in [m/s]')
title('Geschwindigkeit x')
subplot(2,2,3)
hold on;
grid on
plot(time, eulerHistory_estimate(2,:), '.', 'Markersize', 3);
plot(time, eulerHistory_true(2,:));
legend('estimate', 'truth');
legend('Location','southwest')
xlabel('Zeit in [s]')
ylabel('Winkel in [deg]')
title('Nick (pitch) Winkel')
subplot(2,2,4)
hold on;
grid on
plot(time, RAD_TO_DEG*estimateHistory(12,:), '.', 'Markersize', 3)
plot(time, RAD_TO_DEG*xHistory(12,:))
legend('IEKF3', 'truth');
legend('Location','southwest')
title('q')
xlabel('Zeit in [s]')
ylabel('Winkelgeschwindigkeit in [deg/s]')
savefig('x-Direction');
% print(fig,'IEKF3Simulation_x_Direction', '-depsc', '-r600')
%% y-Direction
fig = figure()
set(gcf, 'Position', [0, 0, 1000, 1000])
subplot(2,2,1)
hold on;
grid on;
plot(time, estimateHistory(2,:), '.', 'Markersize', 3);
plot(time, xHistory(2,:));
legend('estimate', 'truth');
legend('Location','southwest')
xlabel('Zeit in [s]')
ylabel('Position in [m]')
title('Position y')
subplot(2,2,2)
hold on;
grid on
plot(time, estimateHistory(5,:), '.', 'Markersize', 3)
plot(time, xHistory(5,:));
legend('estimate', 'truth');
legend('Location','southwest')
xlabel('Zeit in [s]')
ylabel('Geschwindigkeit in [m/s]')
title('Geschwindigkeit y')
subplot(2,2,3)
hold on;
grid on
plot(time, eulerHistory_estimate(1,:), '.', 'Markersize', 3);
plot(time, eulerHistory_true(1,:));
legend('estimate', 'truth');
legend('Location','southwest')
xlabel('Zeit in [s]')
ylabel('Winkel in [deg]')
title('Roll (roll) Winkel')
subplot(2,2,4)
hold on;
grid on
plot(time, RAD_TO_DEG*estimateHistory(11,:), '.', 'Markersize', 3)
plot(time, RAD_TO_DEG*xHistory(11,:))
legend('estimate', 'truth');
legend('Location','southwest')
title('p')
xlabel('Zeit in [s]')
ylabel('Winkelgeschwindigkeit in [deg/s]')
savefig('y-Direction');
% print(fig,'IEKF3Simulation_y_Direction', '-depsc', '-r600')
%% z-Direction
fig = figure();
set(gcf, 'Position', [0, 0, 1000, 1000])
subplot(2,2,1)
hold on;
grid on;
plot(time, estimateHistory(3,:), '.', 'Markersize', 3);
plot(time, xHistory(3,:));
legend('estimate', 'truth');
legend('Location','southwest')
xlabel('Zeit in [s]')
ylabel('Position in [m]')
title('Position z')
subplot(2,2,2)
hold on;
grid on
plot(time, estimateHistory(6,:), '.', 'Markersize', 3)
plot(time, xHistory(6,:));
legend('estimate', 'truth');
legend('Location','southwest')
xlabel('Zeit in [s]')
ylabel('Geschwindigkeit in [m/s]')
title('Geschwindigkeit z')
subplot(2,2,3)
hold on;
grid on
plot(time, eulerHistory_estimate(3,:), '.', 'Markersize', 3);
plot(time, eulerHistory_true(3,:));
legend('estimate', 'truth');
legend('Location','southwest')
xlabel('Zeit in [s]')
ylabel('Winkel in [deg]')
title('Gier (Yaw) Winkel')
subplot(2,2,4)
hold on;
grid on
plot(time, RAD_TO_DEG*estimateHistory(13,:), '.', 'Markersize', 3)
plot(time, RAD_TO_DEG*xHistory(13,:))
legend('estimate', 'truth');
legend('Location','southwest')
title('r')
xlabel('Zeit in [s]')
ylabel('Winkelgeschwindigkeit in [deg/s]')
savefig('z-Direction');
% print(fig,'IEKF3Simulation_z_Direction', '-depsc', '-r600')

%% bias Moments
fig = figure();
set(gcf,'units','points','position',[0,0,500,500])
subplot(3,1,1)
hold on;
grid on;
plot(time, estimateHistory(14,:),'.','MarkerSize',3)
plot([time(1), time(end)], [nb1, nb1]);
plot([time(1), time(end)], [nb1, nb1]+forceMoments(2));
title({'$n_{b,x}$'},'Interpreter','latex')
ylim([0,2.5e-3])
hleg = legend('EKF', 'zugeführt', 'erwartet');
rect = [0.85, 0.9, 0, 0];
set(hleg, 'Position', rect)

subplot(3,1,2)
hold on;
grid on;
plot(time, estimateHistory(15,:),'.','MarkerSize',3)
plot([time(1), time(end)], [nb2, nb2]);
plot([time(1), time(end)], [nb2, nb2]+forceMoments(3));
ylabel('Moment in [Nm]')
title({'$n_{b,y}$'},'Interpreter','latex')
ylim([0,2.5e-3]);
subplot(3,1,3)
hold on;
grid on;
plot(time, estimateHistory(16,:),'.','MarkerSize',3)
plot([time(1), time(end)], [nb3, nb3]);
plot([time(1), time(end)], [nb3, nb3]+forceMoments(4));
xlabel('Zeit in [s]')
title({'$n_{b,z}$'},'Interpreter','latex')
ylim([-1e-3, 1.5e-3])
% print(fig,'IEKF3Simulation_biasMoments', '-depsc', '-r600')
%% motor cmds
fig = figure()
hold on;
grid on;
plot(time, inputHistory);
legend('cp1', 'cp2', 'cp3', 'cp4');
legend('Location','southeast')
title('Motorbefehle');
xlabel('Zeit in s');
ylabel('Motorbefehl in [uint16]')
savefig('Motorbefehle');
% print(fig,'IEKF3Simulation_Motorbefehle', '-depsc', '-r600')

