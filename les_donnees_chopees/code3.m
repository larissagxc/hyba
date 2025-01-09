% Extraction des variables d'intérêt
gps_psi = data{:, 37}; % ψ (orientation globale du bateau)

% Chargement des données (IMU et GPS)
imu_ax = data10{:, 11}; % Accélération X de l'IMU
imu_ay = data10{:, 12}; % Accélération Y de l'IMU
imu_rz = data10{:, 16}; % Vitesse angulaire Z de l'IMU

gps_lat = data1{:, 34}; % Latitude du GPS
gps_lon = data1{:, 35}; % Longitude du GPS
gps_speed = data1{:, 36}; % Vitesse du GPS

% Taille de la fenêtre pour la moyenne mobile
windowSize = 5;

% Application de la moyenne mobile sur les données IMU
imu_ax_filtered = movmean(imu_ax, windowSize);
imu_ay_filtered = movmean(imu_ay, windowSize);
imu_rz_filtered = movmean(imu_rz, windowSize);

% Application de la moyenne mobile sur les données GPS
gps_lat_filtered = movmean(gps_lat, windowSize);
gps_lon_filtered = movmean(gps_lon, windowSize);
gps_speed_filtered = movmean(gps_speed, windowSize);
gps_psi_filtered = movmean(gps_psi, windowSize);

% Synchronisation des temps
gps_time = datetime(data1{:, 1}, 'InputFormat', 'dd/MM/yyyy - HH:mm:ss.S');
imu_time = datetime(data10{:, 1}, 'InputFormat', 'dd/MM/yyyy - HH:mm:ss.S');

% Conversion des temps en secondes depuis le début
gps_time = seconds(gps_time - gps_time(1));
imu_time = seconds(imu_time - imu_time(1));

% S'assurer de la cohérence des données et supprimer les valeurs invalides
valid_idx = isfinite(gps_time) & isfinite(gps_lat_filtered);
gps_time = gps_time(valid_idx);
gps_lat_filtered = gps_lat_filtered(valid_idx);
gps_lon_filtered = gps_lon_filtered(valid_idx);
gps_speed_filtered = gps_speed_filtered(valid_idx);
gps_psi_filtered = gps_psi_filtered(valid_idx);

% Interpolation des données GPS filtrées pour les aligner avec les données IMU
gps_lat_interp = interp1(gps_time, gps_lat_filtered, imu_time, 'linear', 'extrap');
gps_lon_interp = interp1(gps_time, gps_lon_filtered, imu_time, 'linear', 'extrap');
gps_speed_interp = interp1(gps_time, gps_speed_filtered, imu_time, 'linear', 'extrap');
gps_psi_interp = interp1(gps_time, gps_psi_filtered, imu_time, 'linear', 'extrap');

% Affichage des résultats de l'interpolation
figure;

% Latitude
subplot(4,1,1);
plot(gps_time, gps_lat_filtered, 'b--', 'LineWidth', 1.5); hold on;
plot(imu_time, gps_lat_interp, 'r', 'LineWidth', 1);
title('Interpolation - Latitude');
legend('Filtré (GPS)', 'Interpolé');
grid on;

% Longitude
subplot(4,1,2);
plot(gps_time, gps_lon_filtered, 'b--', 'LineWidth', 1.5); hold on;
plot(imu_time, gps_lon_interp, 'r', 'LineWidth', 1);
title('Interpolation - Longitude');
legend('Filtré (GPS)', 'Interpolé');
grid on;

% Vitesse
subplot(4,1,3);
plot(gps_time, gps_speed_filtered, 'b--', 'LineWidth', 1.5); hold on;
plot(imu_time, gps_speed_interp, 'r', 'LineWidth', 1);
title('Interpolation - Vitesse GPS');
legend('Filtré (GPS)', 'Interpolé');
grid on;

% Orientation (ψ)
subplot(4,1,4);
plot(gps_time, gps_psi_filtered, 'b--', 'LineWidth', 1.5); hold on;
plot(imu_time, gps_psi_interp, 'r', 'LineWidth', 1);
title('Interpolation - ψ (Orientation GPS)');
legend('Filtré (GPS)', 'Interpolé');
grid on;
