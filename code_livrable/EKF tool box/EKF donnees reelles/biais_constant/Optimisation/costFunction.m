function err = costFunction(x,consigne,mesure,t,dt)

    % EKF initialisation
    ekf = extendedKalmanFilter(@stateTransitionFcn, @measurementFcn);
    % Bruits et états initiaux
    ekf.State = [mesure(:,1)', x(1),x(2),x(3),x(4)]; % conditions initiales sont actuellement optimisées
    % On peut modifier le code pour que se soit les termes des matrices de
    % pondération
    Q = diag([1e-2,1e-2, 1, 10, 0.2, 1e-6, 1e-6]);
    R = diag([0.05,0.05,0.5]);
    ekf.ProcessNoise = Q; % Bruit de processus
    ekf.MeasurementNoise = R; % Bruit de mesure
    
    %Simulation
    xEstimated = zeros(7, length(t));
    disp(x)
    disp(Q)
    for k = 1:length(t)
        % Commandes actuelles
        current_u = consigne(:,k);
        z = mesure(:,k);
        % Update EKF
        predict(ekf, current_u, dt); % Predict using current commands
        correct(ekf, z); % Correct using measurements   
        % Store results
        xEstimated(:, k) = ekf.State;
    end

    u_est = xEstimated(3,:);
    v_est = xEstimated(4,:);
    psi_est = xEstimated(5,:);
    bu_est = xEstimated(6,:);
    bv_est = xEstimated(7,:);

    Fc = 0.05;
    ts = 1;
    [b,a] = butter(4,Fc*ts*2,"low");
    
    v_est_filt = filtfilt(b, a, v_est);
    psi_est_filt = filtfilt(b, a, psi_est);

    x_cal = zeros(1,length(t));
    y_cal = zeros(1,length(t));
    
    for i = 1:length(t)-1
        x_cal(i+1) = x_cal(i) + ((u_est(i)+bu_est(i))*cos(psi_est_filt(i)) - (v_est_filt(i)+bv_est(i))*sin(psi_est_filt(i)))*dt;
        y_cal(i+1) = y_cal(i) + ((u_est(i)+bu_est(i))*sin(psi_est_filt(i)) + (v_est_filt(i)+bv_est(i))*cos(psi_est_filt(i)))*dt;
    end

    err = norm(y_cal-mesure(2,:),2)+norm(x_cal-mesure(1,:),2); % Fonction de cout sur la trajectoire obtenu par rapport au gps
end
