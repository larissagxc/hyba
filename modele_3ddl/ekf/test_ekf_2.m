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

% Bruits et états initiaux
ekf.State = [x(1); y(1); u(1); v(1); psi(1)]; 
ekf.ProcessNoise = diag([0.001, 0.001, 0.001, 0.001, 0.001]);
ekf.MeasurementNoise = diag(var_bruit(1:3)); % Bruit de mesure

%% Main

xEstimated = zeros(5, length(t));

for k = 1:length(t)

    % Commandes 
    uc = [ax(k); ay(k); ra(k)];

    % Mesures
    z = [x(k); y(k); u(k)];
    
    % Update EKF
    predict(ekf, uc, dt); % Predict using current commands
    correct(ekf, z); % Correct using measurements
    
    xEstimated(:, k) = ekf.State;
end

%% Plot results
figure;
subplot(2,1,1);
plot(t, v, 'r',LineWidth=2);
hold on;
plot(t, xEstimated(4,:), '--k',LineWidth=2);
xlabel('Time (s)');
ylabel('v (m/s)');
legend;
title('Reconstruction de v');

subplot(2,1,2);
plot(t, psi, 'r',LineWidth=2);
hold on;
plot(t, xEstimated(5,:), '--k',LineWidth=2);
xlabel('Time (s)');
ylabel('\psi (rad)');
legend;
title('Reconstruction de \psi');

figure;
subplot(3,1,1);
plot(t, x, 'r',LineWidth=2);
hold on;
plot(t, xEstimated(1,:), '--k',LineWidth=2);
xlabel('Time (s)');
ylabel('x (m)');
legend;
title('Reconstruction de x');

subplot(3,1,2);
plot(t, y, 'r',LineWidth=2);
hold on;
plot(t, xEstimated(2,:), '--k',LineWidth=2);
xlabel('Time (s)');
ylabel('y (m)');
legend;
title('Reconstruction de y');

subplot(3,1,3);
plot(t, u, 'r',LineWidth=2);
hold on;
plot(t, xEstimated(3,:), '--k',LineWidth=2);
plot(t, u_b, '--b',LineWidth=1);
xlabel('Time (s)');
ylabel('u (m/s)');
legend;
title('Reconstruction de u');