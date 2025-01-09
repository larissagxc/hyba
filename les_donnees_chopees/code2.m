% Extraction des variables d'intérêt
gps_psi = data{:, 37}; % ψ (orientation globale du bateau)

% Chargement des données (IMU et GPS)
imu_ax = data10{:, 11}; % Remplacez par le nom réel de la variable
imu_ay = data10{:, 12};
imu_rz = data10{:, 16};

gps_lat = data1{:, 34}; % Remplacez par le nom réel de la variable
gps_lon = data1{:, 35};
gps_speed = data1{:, 36};

% Taille de la fenêtre pour la moyenne mobile
windowSize = 10;

% Application de la moyenne mobile sur les données IMU
imu_ax_filtered = movmean(imu_ax, windowSize);
imu_ay_filtered = movmean(imu_ay, windowSize);
imu_rz_filtered = movmean(imu_rz, windowSize);

% Application de la moyenne mobile sur les données GPS
gps_lat_filtered = movmean(gps_lat, windowSize);
gps_lon_filtered = movmean(gps_lon, windowSize);
gps_speed_filtered = movmean(gps_speed, windowSize);
gps_psi_filtered = movmean(gps_psi, windowSize);

% Affichage des résultats (IMU)
figure;
subplot(3,1,1);
plot(imu_ax, 'r--'); hold on;
plot(imu_ax_filtered, 'b', 'LineWidth', 1.5);
title('Filtre de Moyenne Mobile - Accélération X');
legend('Original', 'Filtré');
grid on;

subplot(3,1,2);
plot(imu_ay, 'r--'); hold on;
plot(imu_ay_filtered, 'b', 'LineWidth', 1.5);
title('Filtre de Moyenne Mobile - Accélération Y');
legend('Original', 'Filtré');
grid on;

subplot(3,1,3);
plot(imu_rz, 'r--'); hold on;
plot(imu_rz_filtered, 'b', 'LineWidth', 1.5);
title('Filtre de Moyenne Mobile - Vitesse Angulaire Z');
legend('Original', 'Filtré');
grid on;

% Affichage des résultats (GPS)
figure;
subplot(4,1,1);
plot(gps_lat, 'r--'); hold on;
plot(gps_lat_filtered, 'b', 'LineWidth', 1.5);
title('Filtre de Moyenne Mobile - Latitude');
legend('Original', 'Filtré');
grid on;

subplot(4,1,2);
plot(gps_lon, 'r--'); hold on;
plot(gps_lon_filtered, 'b', 'LineWidth', 1.5);
title('Filtre de Moyenne Mobile - Longitude');
legend('Original', 'Filtré');
grid on;

subplot(4,1,3);
plot(gps_speed, 'r--'); hold on;
plot(gps_speed_filtered, 'b', 'LineWidth', 1.5);
title('Filtre de Moyenne Mobile - Vitesse GPS');
legend('Original', 'Filtré');
grid on;

subplot(4,1,4);
plot(gps_psi, 'r--'); hold on;
plot(gps_psi_filtered, 'b', 'LineWidth', 1.5);
title('Filtre de Moyenne Mobile - \psi (Orientation GPS)');
legend('Original', 'Filtré');
grid on;
