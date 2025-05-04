function xT = transformRigid3D(x, t)
    % Applies a rigid transformation to a list of 3D points x
    % t = [tx, ty, tz, rx, ry, rz] (translations + rotations in radians)
    
    % Convert to homogeneous coordinates
    X = [x'; ones(1, size(x, 1))];
    
    % Translation matrix
    T = eye(4);
    T(1:3, 4) = t(1:3)';
    
    % Rotation matrices
    Rx = [1, 0, 0;
          0, cos(t(4)), -sin(t(4));
          0, sin(t(4)),  cos(t(4))];
    
    Ry = [cos(t(5)), 0, sin(t(5));
          0, 1, 0;
         -sin(t(5)), 0, cos(t(5))];
    
    Rz = [cos(t(6)), -sin(t(6)), 0;
          sin(t(6)),  cos(t(6)), 0;
          0, 0, 1];
    
    % Combined rotation
    R = Rx * Ry * Rz;
    T(1:3, 1:3) = R;
    
    % Apply transformation
    xT = (T * X)';
    xT = xT(:, 1:3);
    end
    