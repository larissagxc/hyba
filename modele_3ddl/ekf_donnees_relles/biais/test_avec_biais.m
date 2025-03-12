clc
clear 
close all

%% Data initialisation
% load('data_brut_bateau.mat')
% 
% data = [x, y, u, ax, ay, rot_speed_axe_Z];
% legend = 'x, y, u, ax, ay, r';

load('donnees_filtrees_echantillonnees(1).mat')
x = gps_filt.x;
y = gps_filt.y;
u = gps_filt.v_loch_butter;
ax = imu_filt_sous.ax_butter;
ay = imu_filt_sous.ay_butter;
rot_speed_axe_Z = imu_filt_sous.gyro_z_butter;


lat0 = y(1); % Latitude de referência
lon0 = x(1); % Longitude de referência

dlat = (y - lat0) * (pi/180); % Diferença em radianos
dlon = (x - lon0) * (pi/180);

R_terra = 6378137; % Raio médio da Terra em metros
xm = R_terra * dlon * cosd(lat0); % x em metro

u = u*0.5144444444444444444444444;

ym = R_terra * dlat; % y em metros

dt = 1; 
T_final = length(x)-1;
t = 0:dt:T_final;  
psi0 = atan2(ym(2),xm(2));

vx = gradient(xm,1);
vy = gradient(ym,1);

%% EKF initialisation

ekf = extendedKalmanFilter(@stateTransitionFcn, @measurementFcn);

% Initial guesses for the state vector and noise
% ekf.State = [xm(1); ym(1); u(1); 0; psi0; 0; 0]; % Initial state estimate [x, y, u, v, psi]
ekf.State = [1; 1; 1; 3; 0.1; 1e-6; 1e-6];
ekf.ProcessNoise = diag([1,1,1,10,0.1, 1e-6,1e-6]); % Process noise covariance
ekf.MeasurementNoise = diag([0.5,0.5,0.05]); % Measurement noise covariance

%% Main

% Commandes 
au = ax';
av = ay';
r = rot_speed_axe_Z';
r = r*pi/180;

% Initial state
xTrue = [xm(1); ym(1); u(1); 0; psi0; 0; 0]; % [x, y, u, v, psi, bu, bv]
xEstimated = zeros(7, length(t));
xTrueHistory = zeros(7, length(t));

% Initial EKF state
ekf.State = [xm(1); ym(1); u(1); 0; psi0; 0; 0]; % Initial state estimate

for k = 1:length(t)
    % Commandes actuelles
    current_u = [au(k); av(k); r(k)];
    z = [xm(k);ym(k);u(k)];
    % Update EKF
    predict(ekf, current_u, dt); % Predict using current commands
    xTrue(:,k) = ekf.State;
    correct(ekf, z); % Correct using measurements   
    % Store results
    xEstimated(:, k) = ekf.State;
end

%% Plot results
figure;
subplot(2,1,1);
% plot(t, v, '--k', 'DisplayName', 'Measured v',LineWidth=2);
plot(t, xEstimated(4,:), 'b', 'DisplayName', 'Estimated v',LineWidth=2);
hold on 
plot(t, xTrue(4,:), 'b', 'DisplayName', 'Estimated v',LineWidth=2);
xlabel('Time (s)');
ylabel('v (m/s)');
legend;
title('Reconstruction de v');

psi_rad = mod(xEstimated(5,:) + pi, 2*pi) - pi;
psi_rad1 = mod(xTrue(5,:) + pi, 2*pi) - pi;

subplot(2,1,2);
% plot(t, psi, '--k', 'DisplayName', 'Measured v',LineWidth=2);
plot(t, psi_rad, 'b', 'DisplayName', 'Estimated \psi',LineWidth=2);
hold on 
plot(t, psi_rad1, 'b', 'DisplayName', 'Estimated \psi',LineWidth=2);
xlabel('Time (s)');
ylabel('\psi (rad)');
legend;
title('Reconstruction de \psi');

figure;
subplot(3,1,1);
plot(t, xm, 'k', 'DisplayName', 'Measured v',LineWidth=2);
hold on
plot(t, xTrue(1,:), '--g', 'DisplayName', 'Estimated v',LineWidth=2);
plot(t, xEstimated(1,:), '--b', 'DisplayName', 'Estimated v',LineWidth=2);
xlabel('Time (s)');
ylabel('x (m)');
legend;
title('Reconstruction de x');

subplot(3,1,2);
plot(t, ym, 'k', 'DisplayName', 'Measured v',LineWidth=2);
hold on
plot(t, xTrue(2,:), '--g', 'DisplayName', 'Estimated v',LineWidth=2);
plot(t, xEstimated(2,:), '--b', 'DisplayName', 'Estimated \psi',LineWidth=2);
xlabel('Time (s)');
ylabel('y (m)');
legend;
title('Reconstruction de y');

subplot(3,1,3);
plot(t, u, 'k', 'DisplayName', 'Measured v',LineWidth=2);
hold on 
plot(t, xTrue(3,:), '--g', 'DisplayName', 'Estimated v',LineWidth=2);
plot(t, xEstimated(3,:), '--b', 'DisplayName', 'Estimated \psi',LineWidth=2);
xlabel('Time (s)');
ylabel('\psi (rad)');
legend;
title('Reconstruction de u');

figure;
subplot(2,1,1);
% plot(t, v, '--k', 'DisplayName', 'Measured v',LineWidth=2);
plot(t, xEstimated(6,:), 'b', 'DisplayName', 'Estimated b_u',LineWidth=2);
xlabel('Time (s)');
ylabel('b_u');
legend;
title('Baias sur u');

subplot(2,1,2);
% plot(t, psi, '--k', 'DisplayName', 'Measured v',LineWidth=2);
plot(t, xEstimated(7,:), 'b', 'DisplayName', 'Estimated b_v',LineWidth=2);
xlabel('Time (s)');
ylabel('b_v');
legend;
title('Biais sur v');

%% Simulation 

L = 100;

x_est = xEstimated(1,:);
y_est = xEstimated(2,:);
u_est = xEstimated(3,:);
v_est = xEstimated(4,:);
beta_est = atan2(v_est,u_est);
psi_est = xEstimated(5,:);

%%

Fc = 0.05;
ts = 1;

[b,a] = butter(4,Fc*ts*2,"low");

v_est_filt = filtfilt(b, a, v_est);
psi_est_filt = filtfilt(b, a, psi_est);

figure;
subplot(2,1,1);
plot(t, xTrue(4,:), 'k', 'DisplayName', 'v modèle',LineWidth=2);
hold on 
plot(t, v_est, 'b', 'DisplayName', 'v estimée',LineWidth=2);
plot(t, v_est_filt, 'r', 'DisplayName', 'v estimée puis filtrée',LineWidth=2);
xlabel('Time (s)');
ylabel('v (m/s)');
legend('FontSize',20);
title('Reconstruction de la vitesse latérale v','FontSize',20);


subplot(2,1,2);
plot(t, xTrue(5,:), 'k', 'DisplayName', '\psi modèle',LineWidth=2);
hold on 
plot(t, psi_est, 'b', 'DisplayName', '\psi estimé',LineWidth=2);
plot(t, psi_est_filt, 'r', 'DisplayName', '\psi estimé puis filtré',LineWidth=2);
xlabel('Time (s)');
ylabel('\psi (rad)');
legend('FontSize',20);
title('Reconstruction de l''angle de cap \psi','FontSize',20);

%%

beta_est_filt = atan2(v_est_filt,u_est);

for i = 1:length(t)
    figure(100),
    subplot(3,2,1),
    plot(t,u,'k',LineWidth=2)
    hold on
    plot(t,u_est,'--r',LineWidth=2)
    hold off
    title('Vitesse longitudinale u','FontSize',20)
    legend('u mesurée','u estimée','FontSize',15)
    xlabel('Temps en s','FontSize',15)
    ylabel('Vitesse en m/s','FontSize',15)
    grid on

    subplot(3,2,3),
    plot(t,v_est,'--r',LineWidth=2)
    hold on
    plot(t,v_est_filt,'g',LineWidth=2)
    hold off
    title('Reconstruction de la vitesse latérale v','FontSize',20)
    legend('v estimée','v estimée filtrée','FontSize',15)
    xlabel('Temps en s','FontSize',15)
    ylabel('Vitesse en m/s','FontSize',15)
    grid on

    subplot(3,2,5),
    plot(t,psi_est,'--r',LineWidth=2)
    hold on
    plot(t,psi_est_filt,'g',LineWidth=2)
    hold off
    title('Reconstruction de l''angle de cap \psi','FontSize',20)
    legend('\psi estimée','\psi estimée filtrée','FontSize',15)
    xlabel('Temps en s','FontSize',15)
    ylabel('Angle en rad','FontSize',15)
    grid on

    subplot(1,2,2),
    plot(ym,xm,'k','DisplayName','Trajectoire mesurée',LineWidth=2)
    hold on
    plot(y_est,x_est,'--r','DisplayName','Trajectoire estimée',LineWidth=2)
    
    bat = [-L/2, 0 , L/2];
    x_bat = x_est(i)+bat.*cos(psi_est_filt(i)+beta_est_filt(i));
    y_bat = y_est(i)+bat.*sin(psi_est_filt(i)+beta_est_filt(i));
    plot(y_bat,x_bat,'b','DisplayName','Représentation du bateau',LineWidth=6);
    hold off
    axis equal
    legend('FontSize',15)
    title('Trajectoire du bateau','FontSize',20)
    xlabel('Y en m','FontSize',15)
    ylabel('X en m','FontSize',15)
end

% bateau_img = imread('bateau_cool3.png'); % Assurez-vous d’avoir un PNG/JPG

% for i = 1:length(t)
%     figure(100),
%     clf;
% 
%     % Vitesse latérale
%     subplot(2,2,1),
%     plot(t,v_est_filt,'k',LineWidth=2)
%     hold on
%     plot(t(i),v_est_filt(i),'or',LineWidth=2)
%     hold off
%     title('Vitesse latérale v')
%     xlabel('Temps en s')
%     ylabel('Vitesse en m/s')
%     grid on
% 
%     % Angle de cap
%     subplot(2,2,3),
%     plot(t,psi_est_filt,'k',LineWidth=2)
%     hold on
%     plot(t(i),psi_est_filt(i),'or',LineWidth=2)
%     hold off
%     title('Angle de cap \psi')
%     xlabel('Temps en s')
%     ylabel('Angle en °')
%     grid on
% 
%     % Trajectoire + image du bateau
%     subplot(1,2,2),
%     plot(y,x,'k',LineWidth=2)
%     hold on
%     plot(y_est,x_est,'--r',LineWidth=2)
% 
%     % Position du bateau
%     x_bateau = x_est(i);
%     y_bateau = y_est(i);
% 
%     % Rotation de l'image du bateau
%     angle_bateau = rad2deg(psi_est_filt(i) + beta_est_filt(i)); % Convertir en degrés
%     bateau_img_rot = imrotate(bateau_img, -angle_bateau, 'bilinear', 'crop'); 
% 
%     % Taille de l'image
%     scale = L/2; % Ajuster la taille de l'image selon le modèle
%     image([y_bateau-scale, y_bateau+scale], [x_bateau+scale, x_bateau-scale], bateau_img_rot);
% 
%     hold off
%     axis equal
%     title('Trajectoire du bateau','FontSize',20)
%     xlabel('Y en m','FontSize',15)
%     ylabel('X en m','FontSize',15)
% 
%     pause(0.1) % Pause pour animer
% end

%%

Fc = 0.1;
ts = 1;

[b,a] = butter(4,Fc*ts*2,"low");

v_est_filt = filtfilt(b, a, v_est);
psi_est_filt = filtfilt(b, a, psi_est);

figure;
subplot(2,1,1);
plot(t, v_est, 'b', 'DisplayName', 'Estimated v',LineWidth=2);
hold on 
plot(t, v_est_filt, 'r', 'DisplayName', 'Estimated v',LineWidth=2);
xlabel('Time (s)');
ylabel('v (m/s)');
legend;
title('Reconstruction de v');

psi_rad = mod(psi_est + pi, 2*pi) - pi;
psi_rad1 = mod(psi_est_filt + pi, 2*pi) - pi;

subplot(2,1,2);
plot(t, psi_rad, 'b', 'DisplayName', 'Estimated \psi',LineWidth=2);
hold on 
plot(t, psi_rad1, 'r', 'DisplayName', 'Estimated \psi',LineWidth=2);
xlabel('Time (s)');
ylabel('\psi (rad)');
legend;
title('Reconstruction de \psi');