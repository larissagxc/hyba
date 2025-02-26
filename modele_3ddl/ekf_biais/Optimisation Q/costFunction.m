function err = costFunction(x,initialState,consigne,R,mesure,theorique,t,dt)

    % EKF initialisation
    ekf = extendedKalmanFilter(@stateTransitionFcn, @measurementFcn);
    % Bruits et états initiaux
    ekf.State = initialState; 
    Q = diag(x/100000);
    ekf.ProcessNoise = Q; % Bruit de processus
    ekf.MeasurementNoise = R; % Bruit de mesure
    
    %Simulation
    xEstimated = zeros(5, length(t));
    disp(x)
    disp(Q)
    for k = 1:length(t)
        % Commandes 
        uc = consigne(:,k);
        % Mesures
        z = mesure(:,k); 
        % Update EKF
        predict(ekf, uc, dt); % Predict using current commands
        correct(ekf, z); % Correct using measurements
        
        xEstimated(:, k) = ekf.State;
    end

    err = sum((xEstimated(4,:)-theorique(1,:)).^2);
end
