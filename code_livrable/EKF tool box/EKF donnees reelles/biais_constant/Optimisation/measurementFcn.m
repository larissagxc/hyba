function z = measurementFcn(x)
    % Inputs:
    %   x = [x, y, u, v, psi] (state vector)
    % Output:
    %   z = [x, y, u] (measured states)
    
    z = [x(1); x(2); x(3)]; % Only x, y, and u are measured
end
 