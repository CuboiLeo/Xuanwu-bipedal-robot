function Ad_T = adj_rep(T)
    %% This function finds the adjoint representation of a transformation matrix
    % Given T = (R,p), Ad_T = [R 0; [p]R R]
    R = T(1:3,1:3); % Rotation matrix
    p_brkt = [0 -T(3,4) T(2,4);
           T(3,4) 0 -T(1,4);
           -T(2,4) T(1,4) 0]; % Skew symmetric matrix representation of p
    Ad_T(1:3,1:3) = R;
    Ad_T(1:3,4:6) = zeros(3,3); % Write zero out explicitly for clarity
    Ad_T(4:6,1:3) = p_brkt*R;
    Ad_T(4:6,4:6) = R;
end