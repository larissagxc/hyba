clc
clear 
close all
%%
load('data_brut_bateau.mat')

data = [x, y, u, ax, ay, rot_speed_axe_Z];
legend = 'x, y, u, ax, ay, r';

dt = 1;

vx = gradient(x,dt);
vy = gradient(y,dt);

t = 1:length(x);

%% Filtre 

% Paramètres
Fs = 1; % Fréquence d'échantillonnage (en Hz)
Fc = 0.1;   % Fréquence de coupure (en Hz)

% Conception du filtre Butterworth
[b, a] = butter(6, Fc/(Fs/2), 'low'); % 6 est l'ordre du filtre

% Filtrage du signal
ax_filtre = filtfilt(b, a, ax);
ay_filtre = filtfilt(b, a, ay);
x_filtre = filtfilt(b, a, x);
y_filtre = filtfilt(b, a, y);
u_filtre = filtfilt(b, a, u);
r_filtre = filtfilt(b, a, rot_speed_axe_Z);

%% plots

figure,
plot(t,ax,'k',t,ax_filtre,'--r',LineWidth=2)
xlabel('Time (s)');
ylabel('ax (m/s²)');
legend;

figure,
plot(t,ay,'k',t,ay_filtre,'--r',LineWidth=2)
xlabel('Time (s)');
ylabel('ay (m/s²)');
legend;

figure,
plot(t,x,'k',t,x_filtre,'--r',LineWidth=2)
xlabel('Time (s)');
ylabel('x (m)');
legend;

figure,
plot(t,y,'k',t,y_filtre,'--r',LineWidth=2)
xlabel('Time (s)');
ylabel('y (m)');
legend;

figure,
plot(t,u,'k',t,u_filtre,'--r',LineWidth=2)
xlabel('Time (s)');
ylabel('u (m/s)');
legend;

figure,
plot(t,rot_speed_axe_Z,'k',t,r_filtre,'--r',LineWidth=2)
xlabel('Time (s)');
ylabel('r (rad/s)');
legend;
