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

%% EKF initialisation

ekf = extendedKalmanFilter(@stateTransitionFcn, @measurementFcn);

% Initial guesses for the state vector and noise
ekf.State = [xm(1); ym(1); u(1); 0; psi0]; % Initial state estimate [x, y, u, v, psi]
ekf.ProcessNoise = diag([1,1,1,0.01,0.01]); % Process noise covariance
ekf.MeasurementNoise = diag([1,1,0.01]); % Measurement noise covariance

% ekf.ProcessNoise = diag([1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6]); % Process noise covariance
% ekf.MeasurementNoise = eye(3); % Measurement noise covariance


%% Main

% Commandes 
au = ax';
av = ay';
r = rot_speed_axe_Z';
r = r*pi/180;

% Initial state
xTrue = [xm(1); ym(1); u(1); 0; psi0]; % [x, y, u, v, psi, bu, bv]
xEstimated = zeros(5, length(t));
xTrueHistory = zeros(5, length(t));

% Initial EKF state
ekf.State = [xm(1); ym(1); u(1); 0; psi0]; % Initial state estimate

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
xlabel('Time (s)');
ylabel('v (m/s)');
legend;
title('Reconstruction de v');

psi_rad = mod(xEstimated(5,:) + pi, 2*pi) - pi;

subplot(2,1,2);
% plot(t, psi, '--k', 'DisplayName', 'Measured v',LineWidth=2);
plot(t, psi_rad, 'b', 'DisplayName', 'Estimated \psi',LineWidth=2);
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

%% Simulation 

L = 100;

x_est = xEstimated(1,:);
y_est = xEstimated(2,:);
u_est = xEstimated(3,:);
v_est = xEstimated(4,:);
beta_est = atan2(v_est,u_est);
psi_est = xEstimated(5,:);



for i = 1:length(t)
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
    % plot(ym,xm)
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