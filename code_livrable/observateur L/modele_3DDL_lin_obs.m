clear all
close all
clc

%% Paramètres

load('data_virage.mat')
simdata = ans;

T_final = 1000;	       
h = 0.05; 
t = 0:h:T_final;  

u     = simdata(:,1); 
v     = simdata(:,2);          
p     = rad2deg(simdata(:,3));   
ra     = (simdata(:,4));
phi   = rad2deg(simdata(:,5));
psi   = simdata(:,6);
psid   = psi*180/pi;
tauN  = simdata(:,7);
U     = simdata(:,8);
x     = simdata(:,9);
y     = simdata(:,10);
psi_d = rad2deg(simdata(:,11)); % Commande de l'angle psi sur la simulation

vx = u.*cos(psi)-v.*sin(psi);
vy = u.*sin(psi)+v.*cos(psi);

ax = gradient(u,h);
ay = gradient(v,h);

va = zeros(1,length(t));
phia = zeros(1,length(t));
for i = 2:length(t)
    va(1,i) = va(1,i-1) + ay(i-1)*h;
    phia(1,i) = phia(1,i-1) + ra(i-1)*h;
end

figure(4),
plot(t,v,t,va,'--')

%% Point d'équilibre

x_eq = 0;
y_eq = 0;
u_eq = 8;
v_eq = 1;
psi_eq = 0.1;

%% Matrice
A = [0 0 cos(psi_eq) -sin(psi_eq) -u_eq*sin(psi_eq)-v_eq*cos(psi_eq);
     0 0 sin(psi_eq) cos(psi_eq) u_eq*cos(psi_eq)-v_eq*sin(psi_eq);
     0 0 0 0 0;
     0 0 0 0 0;
     0 0 0 0 0];

B = [0 0 0;
     0 0 0;
     1 0 0;
     0 1 0;
     0 0 1];

C = [1 0 0 0 0;
     0 1 0 0 0;
     0 0 1 0 0];

D = zeros(3);

sys = ss(A,B,C,D);

Ob = obsv(A, C);
r = rank(Ob);

if r == 4

    %On enlève la partie non observable
    sysr = minreal(sys);
    [A, B, C, D] = ssdata(sysr);
    %rank(obsv(A_r, C_r))
    
    %Observateur de Luenberger
    poles_obsv = [-10 -10 -10 -11]; % Pôles plus rapides que ceux du système
    L = place(A', C', poles_obsv)';
    
    A_ob = A - L*C;

    % Initialisation de l'état
    x_0 = [x(1); y(1); u(1); v(1)];

    xr = zeros(4,length(t));
    xr(:,1) = x_0;
    
    xr_chap = zeros(4,length(t));
    xr_chap(:,1) = x_0;

    xe = [x_eq; y_eq; u_eq; v_eq];
    x_0 = x_0 - xe;

elseif r == 5
    %Observateur de Luenberger
    poles_obsv = [-10 -1 -1 -1 -5]; % Pôles plus rapides que ceux du système
    L = place(A', C', poles_obsv)';
    
    A_ob = A - L*C;

    % Initialisation de l'état
    x_0 = [x(1); y(1); u(1); v(1); phi(1)];

    xr = zeros(5,length(t));
    xr(:,1) = x_0;
    
    xr_chap = zeros(5,length(t));
    xr_chap(:,1) = x_0;

    xe = [x_eq; y_eq; u_eq; v_eq; psi_eq];

    x_0 = x_0 - xe;

end


%% Simulation

for i = 2:length(t)
    dx = A*xr(:,i-1) + B*[ax(i-1); ay(i-1); ra(i-1)];
    xr(:,i) = xr(:,i-1) + dx*h;

    dx_chap = A_ob*xr_chap(:,i-1) + B*[ax(i-1); ay(i-1); ra(i-1)] + L*[x(i-1); y(i-1); u(i-1)];
    xr_chap(:,i) = xr_chap(:,i-1) + dx_chap*h;
end



xa = xr(1,:);
yr = xr(2,:);
ur = xr(3,:);
vr = xr(4,:);

xa_chap = xr_chap(1,:);
yr_chap = xr_chap(2,:);
ur_chap = xr_chap(3,:);
vr_chap = xr_chap(4,:);

if r == 5
    psir = xr(5,:);
    psir_chap = xr_chap(5,:);

end

%% Plots
figure(1),
plot(x,y,LineWidth=2)
hold on 
plot(xa_chap,yr_chap,'--',LineWidth=2)
hold off
title('Déplacement')
legend('mesurée','reconstruite')
axis equal
grid on

figure(2),
plot(t,u,LineWidth=2)
hold on
plot(t,ur,'--',LineWidth=2)
plot(t,ur_chap,'--',LineWidth=2)
hold off
title('Vitesse u')
legend('mesurée','reconstruite')

figure(3),
plot(t,v,LineWidth=2)
hold on
plot(t,vr_chap,'--',LineWidth=2)
hold off
title('Vitesse v')
legend('mesurée','reconstruite')

figure(4),
plot(t,psi,LineWidth=2)
hold on
plot(t,psir_chap,'--',LineWidth=2)
hold off
title('Angle \psi')
legend('mesurée','reconstruite')


%%
% % On obtient vr_chap = vy ?????
% figure(5),
% plot(t,vy,t,vr_chap,'--')
% % POURQUOI ????