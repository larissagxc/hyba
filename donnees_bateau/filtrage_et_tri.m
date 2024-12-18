% Extração das variáveis de interesse
ax = data{:, 61}; % Accélération longitudinale
ay = data{:, 62}; % Accélération latérale
u = data{:, 36}; % Vitesse longitudinale (GPS)
rot_speed_axe_Z = data{:, 66}; % taux de lacet (r)
x = data{:, 34}; % x (latitude)
y = data{:, 35}; % y (longitude)
psi = data{:, 37}; % ψ (orientation globale du bateau)

time = (1:length(u))';

% Filtre passe-bas pour attaquer les bruits
fs = 10; % Freq. sampling (à modifier)
fc = 0.2; 
[b, a] = butter(2, fc/(fs/2)); 

% données filtrées
u_filtered = filtfilt(b, a, u); 
ax_filtered = filtfilt(b, a, ax); 
ay_filtered = filtfilt(b, a, ay); 
rot_speed_axe_Z_filtered = filtfilt(b, a, rot_speed_axe_Z); 


results = table(time, x, y, psi, u_filtered, ax_filtered, ay_filtered, rot_speed_axe_Z_filtered, ...
    'VariableNames', {'Time', 'Latitude', 'Longitude', 'Psi', 'U', 'Ax', 'Ay', 'RotationalSpeedZ'});

writetable(results, 'filtered_data.csv');

%Showing the filtered results 

figure;
subplot(2, 2, 1);
plot(time, u_filtered, 'b');
title('Vitesse Longitudinale Filtrée (u)');
xlabel('Temps (s)');
ylabel('Vitesse (m/s)');

subplot(2, 2, 2);
plot(time, ax_filtered, 'r');
title('Accélération Longitudinale Filtrée (ax)');
xlabel('Temps (s)');
ylabel('Accélération (m/s²)');

subplot(2, 2, 3);
plot(time, ay_filtered, 'g');
title('Accélération Latérale Filtrée (ay)');
xlabel('Temps (s)');
ylabel('Accélération (m/s²)');

subplot(2, 2, 4);
plot(time, rot_speed_axe_Z_filtered, 'm');
title('Taux de Lacet Filtré (r)');
xlabel('Temps (s)');
ylabel('Rotation (rad/s)');

figure;
plot(time, psi, 'g');
title('ψ orientation globale du bateau');
xlabel('Temps (s)');
ylabel('Angles (deg)');