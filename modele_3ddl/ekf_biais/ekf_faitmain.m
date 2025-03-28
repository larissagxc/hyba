clear all
close all
clc
%% Paramètres

% load('data2.mat')
% 
% T_final = 600;	       
% h = 0.05; 
% t = 0:h:T_final;  
% 
% u     = simdata(:,1); 
% v     = simdata(:,2);          
% r     = (simdata(:,4));
% psi   = simdata(:,6);
% x     = simdata(:,9);
% y     = simdata(:,10);
% 
% vx = u.*cos(psi)-v.*sin(psi);
% vy = u.*sin(psi)+v.*cos(psi);
% 
% ax = gradient(u,h);
% ay = gradient(v,h);

%% Paramètres 2

load('data_virage.mat')
simdata = ans;
load('bruit_signaux_rééls.mat')

bu = 0;
bv = 0;

% Extraction data SIMnavalvessel
u     = simdata(:,1); 
u_b = u + bu;
v     = simdata(:,2);          
r     = (simdata(:,4));
psi   = simdata(:,6);
x     = simdata(:,9);
y     = simdata(:,10);
T_final = 1000;	 

h = 0.05; 
t = 0:h:T_final;  

vx = u.*cos(psi)-v.*sin(psi);
vy = u.*sin(psi)+v.*cos(psi);

ax = gradient(u,h);
ay = gradient(v,h);


%% EKF - Estimation d'état pour un système non linéaire

f = @(X,U,k) [X(3)*cos(X(5)) - X(4)*sin(X(5));
              X(3)*sin(X(5)) + X(4)*cos(X(5));
              U(1,k) + X(6)/h;
              U(2,k) + X(7)/h;
              U(3,k); 
              0; 
              0];
% Jacobienne de f
F = @(X) [0 0 cos(X(5)) -sin(X(5)) -X(3)*sin(X(5))-X(4)*cos(X(5)) 0 0;
        0 0 sin(X(5))  cos(X(5))  X(3)*cos(X(5))-X(4)*sin(X(5)) 0 0;
        0 0     0          0                    0   1/h    0;
        0 0     0          0                    0   0    1/h;
        0 0     0          0                    0   0    0;
        0 0     0          0                    0   0    0;
        0 0     0          0                    0   0    0];

% Matrices de covariance
Q = diag([1e-5;1e-5;1e-5;1e-5;1e-5;1e-12;1e-12]);
R = diag([0.3;0.3;0.3]);

% Initialisation
X_true = [x(1);y(1);u(1);v(1);psi(1);0;0];        % État initial réel
X_hat = [x(1);y(1);u(1);v(1);psi(1);0;0];         % Estimation initiale
P = 1e-4*eye(7);                              % Covariance initiale

% Simulation
U = [ax ay r]';% Entrée (contrôle)
C = [1 0 0 0 0 0 0;0 1 0 0 0 0 0;0 0 1 0 0 0 0];
Y = zeros(3,length(t));               % Observations mesurées
X_estimated = zeros(7,length(t));     % Estimations
X_true_history = zeros(7,length(t));

for k = 1:length(t)
    % --- Dynamique réelle du système ---
    w_x = sqrt(Q) * randn(7, 1);        % Bruit de processus
    w_y = sqrt(R) * randn(3, 1);        % Bruit de mesure
    
    X_true = X_true + h * (f(X_true,U,k) + w_x);
    Y(:, k) = C*X_true + w_y;
    X_true_history(:, k) = X_true;

    % --- Prédiction (Prediction Step) ---
    L = P*C'*inv(R);
    X_hat = X_hat + h * (f(X_hat,U,k) + L*(Y(:, k) - C*X_hat));         % Prédiction de l'état
    X_estimated(:, k) = X_hat;
    P = are(F(X_hat)',C'*inv(R)*C,Q);
end

x_ekf = X_estimated(1,:);
y_ekf = X_estimated(2,:);
u_ekf = X_estimated(3,:);
v_ekf = X_estimated(4,:);
psi_ekf = X_estimated(5,:);
bu_ekf = X_estimated(6,:);
bv_ekf = X_estimated(7,:);

x_mod = X_true_history(1,:);
y_mod = X_true_history(2,:);
u_mod = X_true_history(3,:);
v_mod = X_true_history(4,:);
psi_mod = X_true_history(5,:);
bu_mod = X_true_history(6,:);
bv_mod = X_true_history(7,:);


% --- Affichage des résultats ---
figure(1)
plot(x, y, 'LineWidth', 2)
hold on 
plot(x_ekf, y_ekf, '--', 'LineWidth', 2)
plot(x_mod, y_mod, ':', 'LineWidth', 2)
hold off
title('Déplacement')
% legend('Mesuré','Reconstruit','Recalculé en théorie')
xlabel('x(m)')
ylabel('y(m)')
axis equal
grid on

figure(2)
plot(t, u, 'LineWidth', 2)
hold on
plot(t, u_ekf, '--', 'LineWidth', 2)
plot(t, u_mod, ':', 'LineWidth', 2)
hold off
grid on
title('Vitesse u')
% legend('Mesuré', 'Reconstruit','Recalculé en théorie')
xlabel('temps(s)')
ylabel('u(m/s)')

figure(3)
plot(t, v, 'LineWidth', 2)
hold on
plot(t, v_ekf, '--', 'LineWidth', 2)
plot(t, v_mod, ':', 'LineWidth', 2)
hold off
grid on
title('Vitesse v')
% legend('Mesuré', 'Reconstruit','Recalculé en théorie')
xlabel('temps(s)')
ylabel('v(m/s)')

figure(4)
plot(t, psi, 'LineWidth', 2)
hold on
plot(t,psi_ekf, '--', 'LineWidth', 2)
plot(t, psi_mod, ':', 'LineWidth', 2)
hold off
grid on
title('Angle \psi')
% legend('Mesuré', 'Reconstruit','Recalculé en théorie')
xlabel('temps(s)')
ylabel('\psi(rad)')

figure(5)
plot(t,bu_ekf, '--', 'LineWidth', 2)
hold on
plot(t, bu_mod, ':', 'LineWidth', 2)
hold off
grid on
title('Angle \psi')
% legend('Mesuré', 'Reconstruit','Recalculé en théorie')
xlabel('temps(s)')
ylabel('\psi(rad)')

figure(6)
plot(t,bv_ekf, '--', 'LineWidth', 2)
hold on
plot(t, bv_mod, ':', 'LineWidth', 2)
hold off
grid on
title('Angle \psi')
% legend('Mesuré', 'Reconstruit','Recalculé en théorie')
xlabel('temps(s)')
ylabel('\psi(rad)')