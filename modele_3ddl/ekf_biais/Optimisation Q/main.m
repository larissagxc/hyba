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

%% Data Noise

% Measurements
x_b = x + sqrt(var_bruit(1)) * randn(length(t),1) + moy_bruit(1);
y_b = y + sqrt(var_bruit(2)) * randn(length(t),1) + moy_bruit(2);
u_b = u + sqrt(var_bruit(3)) * randn(length(t),1) + moy_bruit(3);

% Commands
au_b = ax + sqrt(var_bruit(4)) * randn(length(t),1) + moy_bruit(4);
av_b = ay + sqrt(var_bruit(5)) * randn(length(t),1) + moy_bruit(5);
r_b = ra + sqrt(var_bruit(6)) * randn(length(t),1) + moy_bruit(6);

%% Data preparation

initialState = [x(1); y(1); u(1); v(1); psi(1)];

consignes = [ax'; ay'; ra'];
% consignes = [au_b'; av_b'; r_b'];
mesures = [x_b'; y_b'; u_b'];
realSignals = [v'; psi'];

R = diag(var_bruit(1:3)); % Measurement Noise Matrix

%% Optimisation

lb = [0,0,0,0,0];
ub = [100000,100000,100000,100000,100000];  
A = [];
b = [];
Aeq = [];
beq = [];  

q0 = [0.1, 0.1, 0.1, 0, 0];

q = fmincon(@(q)costFunction(q,initialState,consignes,R,mesures,realSignals,t,dt),q0,A,b,Aeq,beq,lb,ub);

%% 

% EKF initialisation
ekf = extendedKalmanFilter(@stateTransitionFcn, @measurementFcn);
% Bruits et états initiaux
ekf.State = initialState; 
ekf.ProcessNoise = diag([1e-6, 1e-6, 1e-6, 0, 0]); % Bruit de processus
ekf.MeasurementNoise = R; % Bruit de mesure

%Simulation
xEstimated = zeros(5, length(t));

    for k = 1:length(t)
        % Commandes 
        uc = consignes(:,k);
        % Mesures
        z = mesures(:,k); 
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
%plot(t, u_b, '--b',LineWidth=1);
xlabel('Time (s)');
ylabel('u (m/s)');
legend;
title('Reconstruction de u');