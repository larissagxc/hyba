clc
clear 
close all

%% Initialisation des données
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
t = 0:dt:T_final;  % Correction ici

% Commandes 
au = ax';
av = ay';
r = rot_speed_axe_Z';
r = r * pi / 180; % Conversion en radians/s

Mesure = [xm'; ym'; u'];
Consigne = [au; av; r];

%% Optimisation

% Définition des bornes de l'optimisation
lb = [-2, -pi, -2, -2]; % limites basses
ub = [2, pi, 2,  2]; % limites hautes
A = [];
b = [];
Aeq = [];
beq = [];  

% Initialisation des paramètres
q0 = [1, 0.3, 0, 0];

% Optimisation
q = fmincon(@(q) costFunction(q, Consigne, Mesure, t, dt), q0, A, b, Aeq, beq, lb, ub);
