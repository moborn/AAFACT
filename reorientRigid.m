function [nodes_final, coords_final, coords_final_unit, Temp_Coordinates_Unit] = reorientRigid(Temp_Nodes_Coords, cm_nodes, side_indx, RTs)
    % Reorients the aligned bone and coordinate system back to its original orientation
    % using inverse of the rigid transformation RTs = [tx, ty, tz, rx, ry, rz]
    
    % Extract origin of coordinate system
    Temp_Coordinates_origin = Temp_Nodes_Coords(end-1,:);
    Temp_Coordinates_temp = Temp_Nodes_Coords(end-5:end,:) - Temp_Coordinates_origin;
    
    % Normalize axes
    Temp_Coordinates_temp = [0 0 0; Temp_Coordinates_temp(2,:)./norm(Temp_Coordinates_temp(2,:));
                             0 0 0; Temp_Coordinates_temp(4,:)./norm(Temp_Coordinates_temp(4,:));
                             0 0 0; Temp_Coordinates_temp(6,:)./norm(Temp_Coordinates_temp(6,:))];
    
    Temp_Coordinates_Unit = Temp_Coordinates_temp + Temp_Coordinates_origin;
    
    %% Inverse Rigid Transform (rotation + translation)
    T = RTs(1:3);         % translation vector
    R_angles = RTs(4:6);  % rotation angles in radians [rx ry rz]
    
    % Rotation matrices
    Rx = [1 0 0;
          0 cos(R_angles(1)) -sin(R_angles(1));
          0 sin(R_angles(1)) cos(R_angles(1))];
    
    Ry = [cos(R_angles(2)) 0 sin(R_angles(2));
          0 1 0;
         -sin(R_angles(2)) 0 cos(R_angles(2))];
    
    Rz = [cos(R_angles(3)) -sin(R_angles(3)) 0;
          sin(R_angles(3)) cos(R_angles(3)) 0;
          0 0 1];
    
    R = Rz * Ry * Rx;
    R_inv = R';
    T_inv = -R_inv * T(:);  % properly rotated inverse translation
    
    % Apply inverse to all node positions
    X = Temp_Nodes_Coords';       % 3 x N
    X_rot = R_inv * X;
    X_final = X_rot + T_inv;
    nodes_coords_final = X_final';
    
    % Also apply inverse to coordinate axes
    X_axes = Temp_Coordinates_Unit';       % 3 x N
    X_axes_rot = R_inv * X_axes;
    X_axes_final = X_axes_rot + T_inv;
    Temp_Coordinates_Unit = X_axes_final';
    
    %% Centering and post-processing
    [nodes_coords_final_centered, ~] = center(nodes_coords_final, 1);
    
    nodes_coords_final = [nodes_coords_final_centered(:,1) + cm_nodes(1), ...
                          nodes_coords_final_centered(:,2) + cm_nodes(2), ...
                          nodes_coords_final_centered(:,3) + cm_nodes(3)];
    
    % Flip if right side
    if side_indx == 1
        nodes_coords_final = nodes_coords_final .* [1, 1, -1];
        Temp_Coordinates_Unit = Temp_Coordinates_Unit .* [1, 1, -1];
    end
    
    % Extract coordinate system from final nodes
    coords_final_origin = nodes_coords_final(end-1,:);
    coords_final_temp = nodes_coords_final(end-5:end,:) - coords_final_origin;
    
    coords_final_temp = [0 0 0; coords_final_temp(2,:)./norm(coords_final_temp(2,:));
                         0 0 0; coords_final_temp(4,:)./norm(coords_final_temp(4,:));
                         0 0 0; coords_final_temp(6,:)./norm(coords_final_temp(6,:))];
    
    coords_final_unit = coords_final_temp + coords_final_origin;
    
    % Output: separate nodes and coordinate system
    nodes_final = nodes_coords_final(1:end-6,:);
    coords_final = nodes_coords_final(end-5:end,:);
    end
    