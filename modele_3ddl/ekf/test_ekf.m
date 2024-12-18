clc
clear 
close all

%% Data initialisation

load('data2.mat')
load('bruit_signaux_rééls.mat')

T_final = 600;	       
dt = 0.05; 
t = 0:dt:T_final;  

u     = simdata(:,1); 
v     = simdata(:,2);          
ra     = (simdata(:,4));
psi   = simdata(:,6);
psid   = psi*180/pi;
U     = simdata(:,8);
x     = simdata(:,9);
y     = simdata(:,10);
psi_d = rad2deg(simdata(:,11)); % Commande de l'angle psi sur la simulation

vx = u.*cos(psi)-v.*sin(psi);
vy = u.*sin(psi)+v.*cos(psi);

ax = gradient(u,dt);
ay = gradient(v,dt);

%% EKF initialisation

ekf = extendedKalmanFilter(@stateTransitionFcn, @measurementFcn);

% Initial guesses for the state vector and noise
ekf.State = [0; 0; 0; 0; 0]; % Initial state estimate [x, y, u, v, psi]
ekf.ProcessNoise = diag([0.01, 0.01, 0.01, 0.01, 0.01]); % Process noise covariance
ekf.MeasurementNoise = diag([0.1, 0.1, 0.1]); % Measurement noise covariance

%% Main

% Commandes 
au = ax;
av = ay;
r = ra;

% Initial state
xTrue = [0; 0; 1; 0; 0]; % [x, y, u, v, psi]
xEstimated = zeros(5, length(t));
xTrueHistory = zeros(5, length(t));

% Initial EKF state
ekf.State = [0; 0; 0; 0; 0]; % Initial state estimate

for k = 1:length(t)
    % Commandes actuelles
    current_u = [au(k); av(k); r(k)];
    
    % Simulate true system (discretized dynamics)
    xTrue(1) = xTrue(1) + (xTrue(3) * cos(xTrue(5)) - xTrue(4) * sin(xTrue(5))) * dt;
    xTrue(2) = xTrue(2) + (xTrue(3) * sin(xTrue(5)) + xTrue(4) * cos(xTrue(5))) * dt;
    xTrue(3) = xTrue(3) + current_u(1) * dt;
    xTrue(4) = xTrue(4) + current_u(2) * dt;
    xTrue(5) = xTrue(5) + current_u(3) * dt;
    
    % Add measurement noise
    z = xTrue(1:3) + randn(3,1) .* var_bruit(1:3)'; % Measurements: [x, y, u] with noise
    
    % Update EKF
    predict(ekf, current_u); % Predict using current commands
    correct(ekf, z); % Correct using measurements
    
    % Store results
    xEstimated(:, k) = ekf.State;
    xTrueHistory(:, k) = xTrue;
end

%% Plot results
figure;
subplot(2,1,1);
plot(t, xTrueHistory(4,:), 'r', 'DisplayName', 'True v');
hold on;
plot(t, v, '--k', 'DisplayName', 'Measured v');
plot(t, xEstimated(4,:), 'b', 'DisplayName', 'Estimated v');
xlabel('Time (s)');
ylabel('v (m/s)');
legend;
title('Reconstruction de v');

subplot(2,1,2);
plot(t, xTrueHistory(5,:), 'r', 'DisplayName', 'True \psi');
hold on;
plot(t, psi, '--k', 'DisplayName', 'Measured v');
plot(t, xEstimated(5,:), 'b', 'DisplayName', 'Estimated \psi');
xlabel('Time (s)');
ylabel('\psi (rad)');
legend;
title('Reconstruction de \psi');
