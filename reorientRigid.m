function [nodes_final, coords_final, coords_final_unit, Temp_Coordinates_Unit] = reorientRigid(Temp_Nodes_Coords, cm_nodes, side_indx, RTs)
    % Reorient aligned bone and coordinate system back to original position
    % RTs = [tx, ty, tz, rx, ry, rz]
    
    % Extract and normalize local coordinate system
    Temp_Coordinates_origin = Temp_Nodes_Coords(end-1,:);
    Temp_Coordinates_temp = Temp_Nodes_Coords(end-5:end,:) - Temp_Coordinates_origin;
    
    Temp_Coordinates_temp = [0 0 0; Temp_Coordinates_temp(2,:)./norm(Temp_Coordinates_temp(2,:));
                             0 0 0; Temp_Coordinates_temp(4,:)./norm(Temp_Coordinates_temp(4,:));
                             0 0 0; Temp_Coordinates_temp(6,:)./norm(Temp_Coordinates_temp(6,:))];
    
    Temp_Coordinates_Unit = Temp_Coordinates_temp + Temp_Coordinates_origin;
    
    % Inverse transform: invert rotation and translation
    inv_RT = [-RTs(1:3), -RTs(4:6)];  % Negate translation and rotation
    nodes_coords_final = transformRigid3DAboutP(Temp_Nodes_Coords, inv_RT, cm_nodes);
    
    % Flip back to right side if needed
    if side_indx == 1
        nodes_coords_final = nodes_coords_final .* [1, 1, -1];
    end
    
    % Extract and normalize coordinate system again
    coods_final_origin = nodes_coords_final(end-1,:);
    coords_final_temp = nodes_coords_final(end-5:end,:) - coods_final_origin;
    
    coords_final_temp = [0 0 0; coords_final_temp(2,:)./norm(coords_final_temp(2,:));
                         0 0 0; coords_final_temp(4,:)./norm(coords_final_temp(4,:));
                         0 0 0; coords_final_temp(6,:)./norm(coords_final_temp(6,:))];
    
    coords_final_unit = coords_final_temp + coods_final_origin;
    
    % Final nodes and coordinate system
    nodes_final = nodes_coords_final(1:end-6,:);
    coords_final = nodes_coords_final(end-5:end,:);
    end
    