clc
clear 
close all

%% Data initialisation
% Extraction des variances caractérisant le bruit
load('bruit_signaux_rééls.mat')

load('data_virage.mat')
simdata = ans; % ligne à ajouter pour data_virage.mat et data_viragelong.mat

% Extraction data SIMnavalvessel
u     = simdata(:,1); 
v     = simdata(:,2);          
ra     = (simdata(:,4));
psi   = simdata(:,6);
x     = simdata(:,9);
y     = simdata(:,10);
T_final = 1000;	 

% Extraction data SIMsupply
% u     = simdata(:,1); 
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

% Commandes 
au = ax;
av = ay;
r = ra;

% Commandes bruitées
% au = ax + randn(1,length(t)) .* sqrt(var_bruit(4));
% av = ay + randn(1,length(t)) .* sqrt(var_bruit(5));
% r = ra + randn(1,length(t)) .* sqrt(var_bruit(6));

%% EKF initialisation

ekf = extendedKalmanFilter(@stateTransitionFcn, @measurementFcn);

% Initial guesses for the state vector and noise
ekf.ProcessNoise = diag([1e-2, 1e-2, 1e-1, 1, 1e-1]); % Matrice Q
ekf.MeasurementNoise = diag([0.01,0.01,0.05]); % Matrice R

%% Main

% Initial state
xTrue = [x(1); y(1); u(1); v(1); psi(1)]; % [x, y, u, v, psi, bu, bv]
xEstimated = zeros(5, length(t));
xTrueHistory = zeros(5, length(t));

% Initial EKF state
ekf.State = [x(1); y(1); u(1); v(1); psi(1)]; % Initial state estimate

for k = 1:length(t)
    % Commandes actuelles
    current_u = [au(k); av(k); r(k)];
    
    % Simulate true system (discretized dynamics)
    xTrue(1) = xTrue(1) + (xTrue(3) * cos(xTrue(5)) - xTrue(4) * sin(xTrue(5))) * dt;
    xTrue(2) = xTrue(2) + (xTrue(3) * sin(xTrue(5)) + xTrue(4) * cos(xTrue(5))) * dt;
    xTrue(3) = xTrue(3) + current_u(1) * dt;
    xTrue(4) = xTrue(4) + current_u(2) * dt;
    xTrue(5) = xTrue(5) + current_u(3) * dt;

    % Mesures non bruitées
    z = [x(k);y(k);u(k)];
    % Mesures bruitées
    % z = [x(k);y(k);u(k)] + randn(3,1) .* sqrt(var_bruit(1:3))';
    
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
plot(t, v, '--k', 'DisplayName', 'Measured v',LineWidth=2);
hold on;
% plot(t, xTrueHistory(4,:), 'r', 'DisplayName', 'True v',LineWidth=2);
plot(t, xEstimated(4,:), 'b', 'DisplayName', 'Estimated v',LineWidth=2);
xlabel('Time (s)');
ylabel('v (m/s)');
legend;
title('Reconstruction de v');

subplot(2,1,2);
plot(t, psi, '--k', 'DisplayName', 'Measured v',LineWidth=2);
hold on;
% plot(t, xTrueHistory(5,:), 'r', 'DisplayName', 'True \psi',LineWidth=2);
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
ylabel('x (m)');
legend;
title('Reconstruction de x');

subplot(3,1,2);
plot(t, xTrueHistory(2,:), 'r', 'DisplayName', 'True \psi',LineWidth=2);
hold on;
plot(t, y, '--k', 'DisplayName', 'Measured v',LineWidth=2);
plot(t, xEstimated(2,:), 'b', 'DisplayName', 'Estimated \psi',LineWidth=2);
xlabel('Time (s)');
ylabel('y (m)');
legend;
title('Reconstruction de y');

subplot(3,1,3);
plot(t, xTrueHistory(3,:), 'r', 'DisplayName', 'True \psi',LineWidth=2);
hold on;
plot(t, u, '--k', 'DisplayName', 'Measured v',LineWidth=2);
plot(t, xEstimated(3,:), 'b', 'DisplayName', 'Estimated \psi',LineWidth=2);
xlabel('Time (s)');
ylabel('u (m/s)');
legend;
title('Reconstruction de u');

%% Simulation 

L = 100;

x_est = xEstimated(1,:);
y_est = xEstimated(2,:);
u_est = xEstimated(3,:);
v_est = xEstimated(4,:);
beta_est = atan2(v_est,u_est);
psi_est = xEstimated(5,:);

%% Reconstruction de la trajectoire à partir des données de sortie du filtre
% Voir modèle cinématique
% Reconstruit à partir des 

x_cal = zeros(1,length(t));
y_cal = zeros(1,length(t));

for i = 1:length(t)-1
    x_cal(i+1) = x_cal(i) + ((u_est(i))*cos(psi_est(i)) - (v_est(i))*sin(psi_est(i)))*dt;
    y_cal(i+1) = y_cal(i) + ((u_est(i))*sin(psi_est(i)) + (v_est(i))*cos(psi_est(i)))*dt;
end

figure;
plot(y_est,x_est,'k',LineWidth=2);
hold on
plot(y_cal,x_cal,'--r',LineWidth=2);
title('Comparaison entre le GPS et la trajectoire reconstruite');
% legend('GPS','Trajectoire reconstruite');

%% Animation
% Affichage de la position, de l'orientation du bateau, des vecteurs
% vitesses et de l'angle de cap
figure(400);
set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 1 1])
hold on
axis equal
grid on

% Initialisation des courbes
h1 = plot(nan, nan, 'k', 'LineWidth', 2); % GPS
h2 = plot(nan, nan, '--r', 'LineWidth', 2); % Reconstruit
h3 = plot(nan, nan, 'b', 'LineWidth', 6); % Bateau
h4 = quiver(0, 0, 0, 0, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Vitesse longitudinale
h5 = quiver(0, 0, 0, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Vitesse latérale
h6 = quiver(0, 0, 0, 0, 0, '--k', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Cap

% legend({'GPS', 'Reconstruit', 'Bateau', 'Vitesse longitudinale', 'Vitesse latérale'}, 'FontSize', 15);
title('Trajectoire du bateau', 'FontSize', 20);
xlabel('Y en m', 'FontSize', 15);
ylabel('X en m', 'FontSize', 15);

for i = 1:10:length(t)
    % Mise à jour des données
    set(h1, 'XData', y, 'YData', x);
    set(h2, 'XData', y_cal, 'YData', x_cal);

    % Calcul des positions du bateau
    bat = [-L/2, 0, L/2];
    x_bat = x_cal(i) + bat .* cos(psi_est(i) + beta_est(i));
    y_bat = y_cal(i) + bat .* sin(psi_est(i) + beta_est(i));
    set(h3, 'XData', y_bat, 'YData', x_bat);

    % Mise à jour des vecteurs de vitesse
    set(h4, 'XData', y_cal(i), 'YData', x_cal(i), ...
        'UData', 50 * (u_est(i)) * sin(psi_est(i) + beta_est(i)), ...
        'VData', 50 * (u_est(i)) * cos(psi_est(i) + beta_est(i)));

    set(h5, 'XData', y_cal(i), 'YData', x_cal(i), ...
        'UData', -50 * (v_est(i)) * cos(psi_est(i) + beta_est(i)), ...
        'VData', 50 * (v_est(i)) * sin(psi_est(i) + beta_est(i)));

    set(h6, 'XData', y_cal(i), 'YData', x_cal(i), ...
            'UData', 100 * sin(psi_est(i)), ...
            'VData', 100 * cos(psi_est(i)));

    xlim([y_cal(i) - 200, y_cal(i) + 200]);
    ylim([x_cal(i) - 200, x_cal(i) + 200]);

    drawnow;
end