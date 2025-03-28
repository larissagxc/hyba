function xNext = stateTransitionFcn(x, uc, dt)
    % États: x = [x, y, u, v, psi, bu, bv] 
    % Commandes: u = [au, av, r] 

    % Extract current states
    xpos = x(1); % Current x position
    ypos = x(2); % Current y position
    u = x(3); % Current velocity in x-direction
    v = x(4); % Current velocity in y-direction
    psi = x(5);   % Current heading angle
    Vc = x(6); % Biais sur la mesure de u supposé constant
    betac = x(7); % Biais sur le modèle de v supposé constant

    bu = -Vc*cos(betac-psi);
    bv = -Vc*sin(betac-psi);

    % Extract inputs (commands)
    au = uc(1); % Acceleration in x-direction
    av = uc(2); % Acceleration in y-direction
    r = uc(3);  % Angular rate

    % Compute the next states using the system dynamics
    xNext = zeros(5, 1);
    xNext(1) = xpos + ((u+bu)* cos(psi) - (v+bv) * sin(psi)) * dt;
    xNext(2) = ypos + ((u+bu) * sin(psi) + (v+bv) * cos(psi)) * dt;
    xNext(3) = u + (au) * dt;
    xNext(4) = v + (av) * dt;
    xNext(5) = psi + r * dt;
    xNext(6) = Vc;
    xNext(7) = betac;
end
