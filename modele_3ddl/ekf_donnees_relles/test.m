clc
clear 


%% Data initialisation
load('data_brut_bateau.mat')

data = [x, y, u, ax, ay, rot_speed_axe_Z];
legend = 'x, y, u, ax, ay, r';

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

%% Main

% Commandes 
au = ax';
av = ay';
r = rot_speed_axe_Z';
r = r*pi/180;

nx = zeros(1,length(t));
ny = zeros(1,length(t));
nu = zeros(1,length(t));
nv = zeros(1,length(t));
npsi = zeros(1,length(t));

nx(1) = xm(1);
ny(1) = ym(1);
nu(1) = u(1);
nv(1) = 0;
npsi(1) = atan2(ym(2),xm(2));;

for i = 1:length(t)-1
    nx(i+1) = nx(i) + (nu(i)*cos(npsi(i)) - nv(i)*sin(npsi(i))) *dt;
    ny(i+1) = ny(i) + (nu(i)*sin(npsi(i)) + nv(i)*cos(npsi(i))) *dt;
    nu(i+1) = nu(i) + au(i) * dt;
    nv(i+1) = nv(i) + av(i) * dt;
    npsi(i+1) = npsi(i) + r(i) * dt;
end

%% Plot results
figure;
subplot(2,1,1);
% plot(t, v, '--k', 'DisplayName', 'Measured v',LineWidth=2);
plot(t, nv, 'b', 'DisplayName', 'Estimated v',LineWidth=2);
xlabel('Time (s)');
ylabel('v (m/s)');
legend;
title('Reconstruction de v');

subplot(2,1,2);
% plot(t, psi, '--k', 'DisplayName', 'Measured v',LineWidth=2);
plot(t, npsi, 'b', 'DisplayName', 'Estimated \psi',LineWidth=2);
xlabel('Time (s)');
ylabel('\psi (rad)');
legend;
title('Reconstruction de \psi');

figure;
subplot(3,1,1);
plot(t, xm, 'k', 'DisplayName', 'Measured v',LineWidth=2);
hold on
plot(t, nx, '--b', 'DisplayName', 'Estimated v',LineWidth=2);
xlabel('Time (s)');
ylabel('x (m)');
legend;
title('Reconstruction de x');

subplot(3,1,2);
plot(t, ym, 'k', 'DisplayName', 'Measured v',LineWidth=2);
hold on
plot(t, ny, '--b', 'DisplayName', 'Estimated \psi',LineWidth=2);
xlabel('Time (s)');
ylabel('y (m)');
legend;
title('Reconstruction de y');

subplot(3,1,3);
plot(t, u, 'k', 'DisplayName', 'Measured v',LineWidth=2);
hold on 
plot(t, nu, '--b', 'DisplayName', 'Estimated \psi',LineWidth=2);
xlabel('Time (s)');
ylabel('\psi (rad)');
legend;
title('Reconstruction de u');