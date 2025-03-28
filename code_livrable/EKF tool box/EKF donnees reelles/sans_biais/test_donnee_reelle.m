clc
clear 
close all

% Chargement des données filtrées
load('donnees_filtrees_echantillonnees(2).mat');

x = gps_filt.x;
y = gps_filt.y;
u = gps_filt.v_loch_butter;
ax = imu_filt_sous.ax_butter;
ay = imu_filt_sous.ay_butter;
rot_speed_axe_Z = imu_filt_sous.gyro_z_butter;

% Conversion des coordonnées GPS en mètres
lat0 = y(1); 
lon0 = x(1);

dlat = (y - lat0) * (pi/180); 
dlon = (x - lon0) * (pi/180);

R_terra = 6378137; % Rayon moyen de la Terre en mètres
xm = R_terra * dlon * cosd(lat0); 
ym = R_terra * dlat;

% Conversion de la vitesse en m/s
u = u * 0.514444;

dt = 1; 
T_final = length(x) - 1;
t = 0:dt:T_final-125;  % Vérifier si -125 est nécessaire 

% Commandes 
au = ax';
av = ay';
r = rot_speed_axe_Z';
r = r*pi/180;

% Conditions initiales
v0 = 0.5;
psi0 = 2.6;

%% EKF initialisation

ekf = extendedKalmanFilter(@stateTransitionFcn, @measurementFcn);

% Initial guesses for the state vector and noise
ekf.ProcessNoise = diag([1e-2,1e-2,1,10,0.5]); % Matrice Q
ekf.MeasurementNoise = diag([0.01, 0.01, 0.05]); % Matrice R

%% Main

% Initial state
% X=[x, y, u, v, psi, bu, bv]
xEstimated = zeros(5, length(t));

% Initial EKF state
ekf.State = [xm(1); ym(1); u(1); 0; psi0]; % Initial state estimate

for k = 1:length(t)
    % Commandes actuelles
    current_u = [au(k); av(k); r(k)];
    z = [xm(k);ym(k);u(k)];
    % Update EKF
    predict(ekf, current_u, dt); % Predict using current commands
    correct(ekf, z); % Correct using measurements   
    % Store results
    xEstimated(:, k) = ekf.State;
end

%% Plot results
figure;
subplot(2,1,1);
plot(t, xEstimated(4,:), 'b', 'DisplayName', 'Estimated v',LineWidth=2);
xlabel('Time (s)');
ylabel('v (m/s)');
legend;
title('Reconstruction de v');

psi_rad = mod(xEstimated(5,:) + pi, 2*pi) - pi;

subplot(2,1,2);
plot(t, psi_rad, 'b', 'DisplayName', 'Estimated \psi',LineWidth=2);
xlabel('Time (s)');
ylabel('\psi (rad)');
legend;
title('Reconstruction de \psi');

figure;
subplot(3,1,1);
plot(t, xm(1:length(t)), 'k', 'DisplayName', 'Measured v',LineWidth=2);
hold on
plot(t, xEstimated(1,:), '--b', 'DisplayName', 'Estimated v',LineWidth=2);
xlabel('Time (s)');
ylabel('x (m)');
legend;
title('Reconstruction de x');

subplot(3,1,2);
plot(t, ym(1:length(t)), 'k', 'DisplayName', 'Measured v',LineWidth=2);
hold on
plot(t, xEstimated(2,:), '--b', 'DisplayName', 'Estimated \psi',LineWidth=2);
xlabel('Time (s)');
ylabel('y (m)');
legend;
title('Reconstruction de y');

subplot(3,1,3);
plot(t, u(1:length(t)), 'k', 'DisplayName', 'Measured v',LineWidth=2);
hold on 
plot(t, xEstimated(3,:), '--b', 'DisplayName', 'Estimated \psi',LineWidth=2);
xlabel('Time (s)');
ylabel('\psi (rad)');
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

%% Filtrage des sorties v et psi

Fc = 0.05;
ts = 1;

% Contruction filtre de butterworth
[b,a] = butter(4,Fc*ts*2,"low");

% Filtrage sans retard
v_est_filt = filtfilt(b, a, v_est);
psi_est_filt = filtfilt(b, a, psi_est);
beta_est_filt = filtfilt(b, a, beta_est);

figure;
subplot(2,1,1);plot(t, v_est, 'b', 'DisplayName', 'v estimée',LineWidth=2);
hold on 
plot(t, v_est_filt, 'r', 'DisplayName', 'v estimée puis filtrée',LineWidth=2);
xlabel('Time (s)');
ylabel('v (m/s)');
legend('FontSize',20);
title('Filtrage v reconstruit','FontSize',20);


subplot(2,1,2);
plot(t, psi_est, 'b', 'DisplayName', '\psi estimé',LineWidth=2);
hold on 
plot(t, psi_est_filt, 'r', 'DisplayName', '\psi estimé puis filtré',LineWidth=2);
xlabel('Time (s)');
ylabel('\psi (rad)');
legend('FontSize',20);
title('Filtrage \psi reconstruit','FontSize',20);

%% Reconstruction de la trajectoire à partir des données de sortie du filtre
% Voir modèle cinématique
% Reconstruit à partir des sorties u, v et psi

x_cal = zeros(1,length(t));
y_cal = zeros(1,length(t));

for i = 1:length(t)-1
    x_cal(i+1) = x_cal(i) + ((u_est(i))*cos(psi_est_filt(i)) - (v_est_filt(i))*sin(psi_est_filt(i)))*dt;
    y_cal(i+1) = y_cal(i) + ((u_est(i))*sin(psi_est_filt(i)) + (v_est_filt(i))*cos(psi_est_filt(i)))*dt;
end

figure,
plot(y_est,x_est,'k',LineWidth=2)
hold on
plot(y_cal,x_cal,'--r',LineWidth=2)
title('Comparaison entre le GPS et la trajectoire reconstruite')
legend('GPS','Trajectoire reconstruite')

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

legend({'GPS', 'Reconstruit', 'Bateau', 'Vitesse longitudinale', 'Vitesse latérale'}, 'FontSize', 15);
title('Trajectoire du bateau', 'FontSize', 20);
xlabel('Y en m', 'FontSize', 15);
ylabel('X en m', 'FontSize', 15);

for i = 1:length(t)
    % Mise à jour des données
    set(h1, 'XData', ym(1:length(t)), 'YData', xm(1:length(t)));
    set(h2, 'XData', y_cal, 'YData', x_cal);

    % Calcul des positions du bateau
    bat = [-L/2, 0, L/2];
    x_bat = x_cal(i) + bat .* cos(psi_est_filt(i) + beta_est_filt(i));
    y_bat = y_cal(i) + bat .* sin(psi_est_filt(i) + beta_est_filt(i));
    set(h3, 'XData', y_bat, 'YData', x_bat);

    % Mise à jour des vecteurs de vitesse
    set(h4, 'XData', y_cal(i), 'YData', x_cal(i), ...
        'UData', 50 * (u_est(i)) * sin(psi_est_filt(i) + beta_est_filt(i)), ...
        'VData', 50 * (u_est(i)) * cos(psi_est_filt(i) + beta_est_filt(i)));

    set(h5, 'XData', y_cal(i), 'YData', x_cal(i), ...
        'UData', 50 * (v_est_filt(i)) * cos(psi_est_filt(i) + beta_est_filt(i)), ...
        'VData', -50 * (v_est_filt(i)) * sin(psi_est_filt(i) + beta_est_filt(i)));

    set(h6, 'XData', y_cal(i), 'YData', x_cal(i), ...
            'UData', 100 * sin(psi_est_filt(i)), ...
            'VData', 100 * cos(psi_est_filt(i)));

    xlim([ym(i) - 200, ym(i) + 200]);
    ylim([xm(i) - 200, xm(i) + 200]);

    drawnow;
end