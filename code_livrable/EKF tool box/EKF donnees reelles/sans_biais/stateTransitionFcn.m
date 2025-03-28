function xNext = stateTransitionFcn(x, uc, dt)
    % États: x = [x, y, u, v, psi] 
    % Commandes: u = [au, av, r] 

    % Extract current states
    xpos = x(1); % Current x position
    ypos = x(2); % Current y position
    u = x(3); % Current velocity in x-direction
    v = x(4); % Current velocity in y-direction
    psi = x(5);   % Current heading angle

    % Extract inputs (commands)
    au = uc(1); % Acceleration in x-direction
    av = uc(2); % Acceleration in y-direction
    r = uc(3);  % Angular rate

    % Compute the next states using the system dynamics
    xNext = zeros(5, 1);
    xNext(1) = xpos + (u * cos(psi) - v * sin(psi)) * dt;
    xNext(2) = ypos + (u * sin(psi) + v * cos(psi)) * dt;
    xNext(3) = u + au * dt;
    xNext(4) = v + av * dt;
    xNext(5) = psi + r * dt;

end
