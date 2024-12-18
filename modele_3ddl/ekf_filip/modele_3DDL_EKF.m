clear all
close all
clc
%% Paramètres

load('data2.mat')

T_final = 600;	       
h = 0.05; 
t = 0:h:T_final;  

u     = simdata(:,1); 
v     = simdata(:,2);          
r     = (simdata(:,4));
psi   = simdata(:,6);
x     = simdata(:,9);
y     = simdata(:,10);

vx = u.*cos(psi)-v.*sin(psi);
vy = u.*sin(psi)+v.*cos(psi);

ax = gradient(u,h);
ay = gradient(v,h);

%% EKF
function XNext = StateTransitionFcn(X, U, dt)
    % X : état courant [x, y, u, v,psi]
    % U : entrée [a_x, a_y, r]
    pos_x = X(1);
    pos_y = X(2);
    u_lin = X(3);
    v_lin = X(4);
    angle_psi = X(5);
    a_x = U(1);
    a_y = U(2);
    r_rot   = U(3);

    
    % Discrétisation des équations
    XNext = zeros(5, 1);
    XNext(1) = pos_x + (u_lin * cos(angle_psi) - v_lin * sin(angle_psi)) * dt; % x
    XNext(2) = pos_y + (u_lin * sin(angle_psi) + v_lin * cos(angle_psi)) * dt; % y
    XNext(3) = u_lin + a_x * dt;  % u
    XNext(4) = v_lin + a_y * dt;  % v
    XNext(5) = angle_psi + r_rot * dt; %psi
end


function Y = MeasurementFcn(X)
    % X : état courant [x, y, u, v, psi]
    Y = [X(1); X(2); X(3)]; % Mesures : [x, y, u]
end

% Paramètres initiaux
%dt = 0.1; % Intervalle de temps
InitialState = [x(1); y(1); u(1); v(1); psi(1)]; % État initial : [x, y, u, v, psi]

% Matrices de covariance
Q = diag([1e-6, 1e-6, 1e-5, 1e-5, 1e-6]); % Bruit du modèle
R = diag([1e-8, 1e-8, 1e-8]);            % Bruit des mesures

% Création de l'EKF
ekf = extendedKalmanFilter(...
    @(X, U) StateTransitionFcn(X, U, h), ... % Fonction de prédiction
    @MeasurementFcn, ...                      % Fonction de mesure
    InitialState);                            % État initial

% Configurer les covariances
ekf.ProcessNoise = Q;
ekf.MeasurementNoise = R;

% Simulation
numSteps = length(t);
trueState = InitialState;
estimatedStates = zeros(5, numSteps);
measurements = zeros(3, numSteps);

for k = 1:numSteps
    % Générer l'état réel avec bruit
    trueState = StateTransitionFcn(trueState, [ax(k);ay(k);r(k)], h);
    measurements(:, k) = MeasurementFcn(trueState) + randn(3, 1) .* sqrt(diag(R));
    
    % EKF : Prédiction et correction
    predict(ekf, [ax(k);ay(k);r(k)]);
    estimatedStates(:, k) = correct(ekf, measurements(:, k));
end

% Reconstruction des états
x_ekf = estimatedStates(1, :);
y_ekf = estimatedStates(2, :);
u_ekf = estimatedStates(3, :);
v_ekf = estimatedStates(4, :);

%% Plots
figure(1),
plot(x,y)
hold on 
plot(x_ekf,y_ekf,'--o')
hold off
title('Déplacement')
legend('mesurée','reconstruite')
axis equal
grid on

figure(2),
plot(t,u)
hold on
% plot(t,ur,'--')
plot(t,u_ekf,'--')
hold off
title('Vitesse u')
legend('mesurée','reconstruite')

figure(3),
plot(t,v)
hold on
% plot(t,vr,'--')
plot(t,v_ekf,'--')
hold off
title('Vitesse v')
legend('mesurée','reconstruite')