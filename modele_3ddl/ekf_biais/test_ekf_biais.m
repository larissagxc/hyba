clc
clear 
close all

%% Data initialisation

load('data_virage.mat')
simdata = ans;
load('bruit_signaux_rééls.mat')

bu = 0;
bv = 0;

% Extraction data SIMnavalvessel
u     = simdata(:,1); 
u_b = u + bu;
v     = simdata(:,2);          
ra     = (simdata(:,4));
psi   = simdata(:,6);
x     = simdata(:,9);
y     = simdata(:,10);
T_final = 1000;	 

% Extraction data SIMsupply
% u     = simdata(:,1); 
% u_b = u + bu;
% v     = simdata(:,2);          
% ra     = simdata(:,3);   
% x     = simdata(:,4);
% y     = simdata(:,5);
% psi   = simdata(:,6);
% T_final = 500;	 
	       
dt = 0.05; 
t = 0:dt:T_final;  

vx = u.*cos(psi)-v.*sin(psi);
vy = u.*sin(psi)+v.*cos(psi);

ax = gradient(u,dt);
ay = gradient(v,dt);

%% EKF initialisation

ekf = extendedKalmanFilter(@stateTransitionFcn, @measurementFcn);

% Initial guesses for the state vector and noise
ekf.State = [x(1); y(1); u(1); v(1); psi(1); 0; 0]; % Initial state estimate [x, y, u, v, psi]
ekf.ProcessNoise = diag([1e-6, 1e-6, 1e-6, 0, 0, 0, 0]); % Process noise covariance
ekf.MeasurementNoise = diag(var_bruit(1:3)); % Measurement noise covariance

% ekf.ProcessNoise = diag([1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6]); % Process noise covariance
% ekf.MeasurementNoise = eye(3); % Measurement noise covariance


%% Main

% Commandes 
au = ax;
av = ay;
r = ra;

% Initial state
xTrue = [x(1); y(1); u(1); v(1); psi(1); 0; 0]; % [x, y, u, v, psi, bu, bv]
xEstimated = zeros(7, length(t));
xTrueHistory = zeros(7, length(t));

% Initial EKF state
ekf.State = [x(1); y(1); u(1); v(1); psi(1); 0; 0]; % Initial state estimate

for k = 1:length(t)
    % Commandes actuelles
    current_u = [au(k); av(k); r(k)];
    
    % Simulate true system (discretized dynamics)
    xTrue(1) = xTrue(1) + (xTrue(3) * cos(xTrue(5)) - xTrue(4) * sin(xTrue(5))) * dt;
    xTrue(2) = xTrue(2) + (xTrue(3) * sin(xTrue(5)) + xTrue(4) * cos(xTrue(5))) * dt;
    xTrue(3) = xTrue(3) + current_u(1) * dt + xTrue(6) ;
    xTrue(4) = xTrue(4) + current_u(2) * dt + xTrue(7);
    xTrue(5) = xTrue(5) + current_u(3) * dt;
    xTrue(6) = xTrue(6);
    xTrue(7) = xTrue(7);
    
    % Add measurement noise
    % z = xTrue(1:3) + randn(3,1) .* sqrt(var_bruit(1:3))'; % Measurements: [x, y, u] with noise

    z = [x(k);y(k);u(k)] + randn(3,1) .* sqrt(var_bruit(1:3))';
    
    % Update EKF
    predict(ekf, current_u, dt); % Predict using current commands
    correct(ekf, z); % Correct using measurements
    
    % Store results
    xEstimated(:, k) = ekf.State;
    xTrueHistory(:, k) = xTrue;
end

%% Plot results
figure;
subplot(2,1,1);
plot(t, xTrueHistory(4,:), 'r', 'DisplayName', 'True v',LineWidth=2);
hold on;
plot(t, v, '--k', 'DisplayName', 'Measured v',LineWidth=2);
plot(t, xEstimated(4,:), 'b', 'DisplayName', 'Estimated v',LineWidth=2);
xlabel('Time (s)');
ylabel('v (m/s)');
legend;
title('Reconstruction de v');

subplot(2,1,2);
plot(t, xTrueHistory(5,:), 'r', 'DisplayName', 'True \psi',LineWidth=2);
hold on;
plot(t, psi, '--k', 'DisplayName', 'Measured v',LineWidth=2);
plot(t, xEstimated(5,:), 'b', 'DisplayName', 'Estimated \psi',LineWidth=2);
xlabel('Time (s)');
ylabel('\psi (rad)');
legend;
title('Reconstruction de \psi');

figure;
subplot(3,1,1);
plot(t, xTrueHistory(1,:), 'r', 'DisplayName', 'True v',LineWidth=2);
hold on;
plot(t, x, '--k', 'DisplayName', 'Measured v');
plot(t, xEstimated(1,:), 'b', 'DisplayName', 'Estimated v',LineWidth=2);
xlabel('Time (s)');
ylabel('v (m/s)');
legend;
title('Reconstruction de x');

subplot(3,1,2);
plot(t, xTrueHistory(2,:), 'r', 'DisplayName', 'True \psi',LineWidth=2);
hold on;
plot(t, y, '--k', 'DisplayName', 'Measured v',LineWidth=2);
plot(t, xEstimated(2,:), 'b', 'DisplayName', 'Estimated \psi',LineWidth=2);
xlabel('Time (s)');
ylabel('\psi (rad)');
legend;
title('Reconstruction de y');

subplot(3,1,3);
plot(t, xTrueHistory(3,:), 'r', 'DisplayName', 'True \psi',LineWidth=2);
hold on;
plot(t, u, '--k', 'DisplayName', 'Measured v',LineWidth=2);
plot(t, xEstimated(3,:), 'b', 'DisplayName', 'Estimated \psi',LineWidth=2);
xlabel('Time (s)');
ylabel('\psi (rad)');
legend;
title('Reconstruction de u');

figure;
plot(t, xTrueHistory(6,:), 'r', 'DisplayName', 'True v',LineWidth=2);
hold on;
plot(t, xEstimated(6,:), 'b', 'DisplayName', 'Estimated v',LineWidth=2);
plot([t(1),t(end)],[bu,bu],'--k',LineWidth=2)
xlabel('Time (s)');
ylabel('v (m/s)');
legend;
title('Reconstruction du biais sur u');

figure;
plot(t, xTrueHistory(7,:), 'r', 'DisplayName', 'True v',LineWidth=2);
hold on;
plot(t, xEstimated(7,:), 'b', 'DisplayName', 'Estimated v',LineWidth=2);
plot([t(1),t(end)],[bv,bv],'--k',LineWidth=2)
xlabel('Time (s)');
ylabel('v (m/s)');
legend;
title('Reconstruction du biais sur v');

%% Simulation 

L = 500;

x_est = xEstimated(1,:);
y_est = xEstimated(2,:);
u_est = xEstimated(3,:);
v_est = xEstimated(4,:);
beta_est = atan2(v_est,u_est);
psi_est = xEstimated(5,:);

a = 1:50:length(t);

for i = a
    figure(100),
    subplot(2,2,1),
    plot(t,beta_est,'k',LineWidth=2)
    hold on
    plot(t(i),beta_est(i),'or',LineWidth=2)
    hold off
    xlim()
    title('Angle de dérive \beta')
    xlabel('Temps en s')
    ylabel('Angle en °')
    grid on

    subplot(2,2,3),
    plot(t,psi_est,'k',LineWidth=2)
    hold on
    plot(t(i),psi_est(i),'or',LineWidth=2)
    hold off
    title('Angle de cap \psi')
    xlabel('Temps en s')
    ylabel('Angle en °')
    grid on

    subplot(1,2,2),
    plot(y,x,'k',LineWidth=2)
    hold on
    plot(y_est,x_est,'--r',LineWidth=2)
    bat = [-L/2, 0 , L/2];
    x_bat = x_est(i)+bat.*cos(psi_est(i)+beta_est(i));
    y_bat = y_est(i)+bat.*sin(psi_est(i)+beta_est(i));
    plot(y_bat,x_bat,'b',LineWidth=2);
    hold off
    axis equal
    title('Trajectoire du bateau')
    xlabel('Y en m')
    ylabel('X en m')
end