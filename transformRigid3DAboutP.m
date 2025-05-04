function xOT = transformRigid3DAboutP(x, t, P)
    % Applies a rigid transform about point P to points x
    % Rotates about P, then translates
    
    % Shift origin to P
    x0 = x - P;
    
    % Apply rigid transformation
    x0T = transformRigid3D(x0, t);
    
    % Shift back
    xOT = x0T + P;
    end
    