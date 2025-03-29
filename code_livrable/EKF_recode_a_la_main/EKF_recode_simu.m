clear all
close all
clc
%% ---Paramétrage simulation---

%Importation données simulation
load('data_virage.mat');
h = 0.05;                     % pas de temps
%ans = simdata;  %à changer en fonction du .mat
T_final = (length(ans)-1)*h;  % durée simulation(s) ATTENTION à l'importation --> peut être différent de ans...
t = 0:h:T_final;

u   = ans(:,1);  % vitesse longitudinale repère bateau (m/s)
v   = ans(:,2);  % vitesse transversale repère bateau (m/s)
r   = ans(:,4);  % lacet (rad/s)
psi = ans(:,6);  % angle de cap (rad)
x   = ans(:,9);  % position x (m)
y   = ans(:,10); % position y (m)

vx = u.*cos(psi) - v.*sin(psi);  % vitesse selon x repère NED
vy = u.*sin(psi) + v.*cos(psi);  % vitesse selon y repère NED

ax = gradient(u,h);  % accélération longitudinale repère bateau
ay = gradient(v,h);  % accélération transversale repère bateau

%% ---Modèle--- 

% Fonction f(X) = f(x,y,u,v,psi)
f = @(X) [X(3)*cos(X(5)) - X(4)*sin(X(5));
          X(3)*sin(X(5)) + X(4)*cos(X(5));
                         0               ;
                         0               ;
                         0              ];
          
% Jacobienne de f(X)
F = @(X) [0 0 cos(X(5)) -sin(X(5)) -(X(3)*sin(X(5)) + X(4)*cos(X(5)));
          0 0 sin(X(5))  cos(X(5))   X(3)*cos(X(5)) - X(4)*sin(X(5)) ;
          0 0    0           0                      0                ;
          0 0    0           0                      0                ;
          0 0    0           0                      0               ];

B = [0 0 0;
     0 0 0;
     1 0 0;
     0 1 0;
     0 0 1];

C = [1 0 0 0 0;
     0 1 0 0 0;
     0 0 1 0 0];

%entrée du système 
U = [ax ay r]'; 

% Matrices de covariance
Q = diag([1e-6; 1e-6; 1e-6; 1e-6; 1e-6]);
R = eye(3);

%Initialisation
X0 = [x(1); y(1); u(1); v(1); psi(1)];           % État initial      
P0 = eye(5);                                     % Covariance initiale

X_bar = X0;
P_bar = P0;

Y = zeros(3, length(t));                         %Sortie système
X_hat = zeros(5, length(t));                     %Etat estimé
X = zeros(5, length(t)+1);                       %Etat
X(:,1) = X0;

for k = 1:length(t)
    w_x = sqrt(Q) * randn(5, 1);                 % Bruit de processus
    w_y = sqrt(R) * randn(3, 1);                 % Bruit de mesure
    Y(:, k) = C * X(:,k) + w_y;                  %Equation de sortie

    K = P_bar * C' / (C * P_bar * C' + R);             %Gain de Kalman
    X_hat(:,k) = X_bar + K * (Y(:, k) - C * X_bar);    %MAJ Estimation
    P_hat = (eye(5)-K*C)*P_bar*(eye(5)-K*C)' + K*R*K'; %MAJ Cov erreur

    X(:,k+1) = X(:,k) + h * (f(X(:,k)) + B*U(:,k) + w_x); % Éq. d'état discrète
    X_bar = X_hat(:,k) +  h * (f(X_hat(:,k)) + B*U(:,k));  %Propagation estimation
    P_bar = (eye(5) + h*F(X_hat(:,k)))*P_hat*(eye(5) + h*F(X_hat(:,k)))' + h^2 *Q; %Propagation Cov erreur
end

%Extraction des états estimés
x_ekf = X_hat(1, :);
y_ekf = X_hat(2, :);
u_ekf = X_hat(3, :);
v_ekf = X_hat(4, :);
psi_ekf = X_hat(5, :);

%Extraction calcul des états d'après le modèle théorique
x_mod = X(1,1:end-1);
y_mod = X(2,1:end-1);
u_mod = X(3,1:end-1);
v_mod = X(4,1:end-1);
psi_mod = X(5,1:end-1);

%% ---Affichage des résultats---

%Trajectoire (x,y)
figure(1)                   
plot(x, y, 'LineWidth', 2)
hold on
plot(x_ekf, y_ekf, '--', 'LineWidth', 2)
plot(x_mod, y_mod, ':', 'LineWidth', 2)
hold off
title('Déplacement du bateau')
legend('Simulation', 'Estimation', 'Théorie')
xlabel('y (m)')
ylabel('x (m)')
axis equal
grid on

%Vitesse longitudinale u(t)
figure(2)
plot(t, u, 'LineWidth', 2)
hold on
plot(t, u_ekf, '--', 'LineWidth', 2)
plot(t, u_mod, ':', 'LineWidth', 2)
hold off
grid on
title('Vitesse u')
legend('Simulation', 'Estimation', 'Théorie')
xlabel('temps(s)')
ylabel('u(m/s)')

%vitesse transversale v(t)
figure(3)
plot(t, v, 'LineWidth', 2)
hold on
plot(t, v_ekf, '--', 'LineWidth', 2)
plot(t, v_mod, ':', 'LineWidth', 2)
hold off
grid on
title('Vitesse v')
legend('Simulation', 'Estimation', 'Théorie')
xlabel('temps(s)')
ylabel('v(m/s)')

%angle de cap psi(t)
figure(4)
plot(t, psi, 'LineWidth', 2)
hold on
plot(t,psi_ekf, '--', 'LineWidth', 2)
plot(t, psi_mod, ':', 'LineWidth', 2)
hold off
grid on
title('Angle \psi')
legend('Simulation', 'Estimation', 'Théorie')  
xlabel('temps(s)')
ylabel('\psi(rad)')

