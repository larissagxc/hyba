% Données d'exemple (remplacez par vos données réelles)
latitude = data1{:,34};  % Latitude en degrés
longitude = data1{:,35}; % Longitude en degrés
cog = data1{:,37};  % GPS COURSE OVER GROUND en degrés

% S'assurer que les vecteurs ont la même taille
if length(latitude) ~= length(longitude)
    error('La latitude et la longitude doivent avoir la même taille.');
end

% Calcul des différences consécutives de latitude et de longitude
delta_lat = diff(latitude);
delta_lon = diff(longitude);

% Estimation du cap (psi) en degrés
psi_estime = atan2d(delta_lon, delta_lat); % Calcule l'angle en degrés
psi_estime = mod(psi_estime, 360); % Normaliser à l'intervalle 0-360 degrés

% Ajuster la taille du COG pour la comparaison (supprimer le dernier élément du COG si nécessaire)
cog_compare = cog(1:end-1); % Ignorer la dernière valeur car psi_estime a une valeur en moins

% Comparaison graphique
figure(2);
plot(psi_estime, 'b', 'LineWidth', 1.5); hold on;
plot(cog_compare, 'r--', 'LineWidth', 1.5);
xlabel('Indice de Temps');
ylabel('Cap (degrés)');
legend('Cap Estimé (\psi)', 'GPS COURSE OVER GROUND (COG)');
title('Comparaison entre le Cap Estimé et le COG');
grid on;

%% finalamente ça n'a pas été une bonne démarche !!!
% on a finit par estimé COG au lieu de psi