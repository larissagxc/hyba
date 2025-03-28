clc
clear 
close all

%%
load('data_brut_bateau.mat')

data = [x, y, u, ax, ay, rot_speed_axe_Z];
legend = 'x, y, u, ax, ay, r';

[b, a] = butter(6, 0.1, 'low'); % Filtre passe-haut

x_exp = filtfilt(b, a, x);
y_exp = filtfilt(b, a, y);
u_exp = filtfilt(b, a, u);

%%
t = 1:length(x_exp);

dt = 1;

% Données expérimentales (x_exp, y_exp, u_exp)
% x_exp, y_exp, u_exp : vecteurs des données expérimentales
% dt : pas d'échantillonnage
N = length(x_exp);

% Calcul des dérivées observées par différences finies
dx_obs = diff(x_exp) / dt; % (N-1)x1
dy_obs = diff(y_exp) / dt; % (N-1)x1
du_obs = diff(u_exp) / dt; % (N-1)x1

% Ajuster les tailles des états expérimentaux
x_exp_trim = x_exp(1:end-1); % États alignés avec dx_obs
y_exp_trim = y_exp(1:end-1);
u_exp_trim = u_exp(1:end-1);

% Calcul des dérivées prédites par le modèle
% Modèle dynamique
psi_exp = atan2(y_exp_trim, x_exp_trim); % Exemple si psi n'est pas mesuré
dx_pred = u_exp_trim .* cos(psi_exp);    % Modèle : dx/dt
dy_pred = u_exp_trim .* sin(psi_exp);    % Modèle : dy/dt
du_pred = zeros(size(u_exp_trim));       % Pas d'accélération (simplification)

% Calcul des résidus
res_x = dx_obs - dx_pred;
res_y = dy_obs - dy_pred;
res_u = du_obs - du_pred;

% Matrice des résidus
residuals = [res_x'; res_y'; res_u']; % (3 x N-1)

% Calcul de la covariance Q
Q = cov(residuals');
