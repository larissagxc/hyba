clc
clear 
close all

%%
load('data_brut_bateau.mat')

data = [x, y, u, ax, ay, rot_speed_axe_Z];
legend = 'x, y, u, ax, ay, r';

%%
figure,
plot(rot_speed_axe_Z)

t = 1:length(x);

%%
[b, a] = butter(6, 0.1, 'high'); % Filtre passe-haut

moy_bruit = zeros(1,6);
var_bruit = zeros(1,6);

for i =1:6  
    bruit = filtfilt(b, a, data(:,i));

    moy_bruit(i) = mean(bruit);
    var_bruit(i) = var(bruit);
end


%%
[b, a] = butter(6, 0.1, 'high'); % Filtre passe-haut
bruit = filtfilt(b, a, ax);

% Moyenne et variance
moyenne_bruit = mean(bruit);
variance_bruit = var(bruit);

% Histogramme pour vérifier la distribution
figure,
histogram(bruit);
title('Histogramme du bruit');

% Simulation du bruit
bruit_simule = sqrt(variance_bruit) * randn(1, length(t)) + moyenne_bruit;

% Densité spectrale de puissance
% [Pxx, f] = pwelch(bruit, [], [], [], Fs);
% plot(f, 10*log10(Pxx));
% title('Spectre de puissance du bruit');
% xlabel('Fréquence (Hz)');
% ylabel('Puissance (dB)');

figure,
histogram(bruit_simule);
title('Histogramme du bruit simulé');

figure,
plot(t,bruit,t,bruit_simule)

%%

% Paramètres
Fs = 1; % Fréquence d'échantillonnage (en Hz)
Fc = 0.05;   % Fréquence de coupure (en Hz)

% Conception du filtre Butterworth
[b, a] = butter(6, Fc/(Fs/2), 'low'); % 6 est l'ordre du filtre

% Filtrage du signal
ax_filtre = filtfilt(b, a, ax);

figure,
plot(t,ax_filtre)
hold on 
plot(t,ax-bruit)