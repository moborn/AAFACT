function [x_opt, data_fitted, rms_vals] = fitRigid(data, bone_indx, bone_coord, varargin)
    % fitRigid fits a rigid transformation (tx, ty, tz, rx, ry, rz) from `data` to `target`.
    %
    % Usage:
    %   [x_opt, data_fitted] = fitRigid(data, target)
    %   [x_opt, data_fitted, rms_vals] = fitRigid(..., 'output_errors', true)
    %
    % Optional Name-Value Parameters:
    %   't0'           - Initial guess (default: [0 0 0 0 0 0])
    %   'xtol'         - Tolerance (default: 1e-3)
    %   'rotcentre'    - Rotation center (default: mean of data)
    %   'maxfev'       - Max function evaluations (default: [])
    %   'maxfun'       - Max total function calls (default: [])
    %   'sample'       - Sample size (default: 10000)
    %   'verbose'      - Verbose output (default: false)
    %   'epsfcn'       - Precision (default: 0)
    %   'output_errors'- Return initial and final RMS (default: false)
    
    % Parse optional arguments
    p = inputParser;
    addParameter(p, 't0', []);
    addParameter(p, 'xtol', 1e-3);
    addParameter(p, 'rotcentre', []);
    addParameter(p, 'maxfev', []);
    addParameter(p, 'maxfun', []);
    addParameter(p, 'sample', 10000);
    addParameter(p, 'verbose', false);
    addParameter(p, 'epsfcn', 0);
    addParameter(p, 'output_errors', false);
    parse(p, varargin{:});
    opts = p.Results;
    scalefactor = 1;
    addpath('AAFACT_utils\Template_Bones\')
    %% Read in Template Bone
    if bone_indx == 1 && bone_coord == 1 % TN
        TR_template = stlread('Talus_Template.stl');
        a = 2;
    elseif bone_indx == 1 && bone_coord >= 2 % TT & ST
        TR_template2 = stlread('Talus_Template2.stl');
        TR_template = stlread('Talus_Template.stl');
        nodes_template2 = TR_template2.Points;
        a = 2;
    elseif bone_indx == 2 && bone_coord == 1
        TR_template = stlread('Calcaneus_Template.stl');
        a = 2;
    elseif bone_indx == 2 && bone_coord == 2
        TR_template = stlread('Calcaneus_Template2.stl');
        a = 2;
    elseif bone_indx == 3
        TR_template = stlread('Navicular_Template.stl');
        a = 1;
    elseif bone_indx == 4
        TR_template = stlread('Cuboid_Template.stl');
        a = 2;
    elseif bone_indx == 5
        TR_template = stlread('Medial_Cuneiform_Template.stl');
        a = 3;
        
    elseif bone_indx == 6
        TR_template = stlread('Intermediate_Cuneiform_Template.stl');
        a = 3;
    elseif bone_indx == 7
        TR_template = stlread('Lateral_Cuneiform_Template.stl');
        a = 3;
    elseif bone_indx == 8
        TR_template = stlread('Metatarsal1_Template.stl');
        tempdata = double(full(data));    
        shp = alphaShape(tempdata);
        % shp = alphaShape(pts, alpha);      % Create alpha shape
        inputvol = volume(shp); 
                % Compute volume
        tempdata = double(full(TR_template.Points));
        shp = alphaShape(tempdata);
        targetvol = volume(shp); 
        scalefactor = inputvol/targetvol;
        a = 2;
    elseif bone_indx == 9
        TR_template = stlread('Metatarsal2_Template.stl');
        tempdata = double(full(data));    
        shp = alphaShape(tempdata);
        % shp = alphaShape(pts, alpha);      % Create alpha shape
        inputvol = volume(shp); 
                % Compute volume
        tempdata = double(full(TR_template.Points));
        shp = alphaShape(tempdata);
        targetvol = volume(shp); 
        scalefactor = inputvol/targetvol;
        a = 2;
    elseif bone_indx == 10
        TR_template = stlread('Metatarsal3_Template.stl');
        tempdata = double(full(data));    
        shp = alphaShape(tempdata);
        % shp = alphaShape(pts, alpha);      % Create alpha shape
        inputvol = volume(shp); 
                % Compute volume
        tempdata = double(full(TR_template.Points));
        shp = alphaShape(tempdata);
        targetvol = volume(shp); 
        scalefactor = inputvol/targetvol;
        a = 2;
    elseif bone_indx == 11
        TR_template = stlread('Metatarsal4_Template.stl');
        tempdata = double(full(data));    
        shp = alphaShape(tempdata);
        % shp = alphaShape(pts, alpha);      % Create alpha shape
        inputvol = volume(shp); 
                % Compute volume
        tempdata = double(full(TR_template.Points));
        shp = alphaShape(tempdata);
        targetvol = volume(shp); 
        scalefactor = inputvol/targetvol;
        a = 2;
    elseif bone_indx == 12
        TR_template = stlread('Metatarsal5_Template.stl');
        tempdata = double(full(data));    
        shp = alphaShape(tempdata);
        % shp = alphaShape(pts, alpha);      % Create alpha shape
        inputvol = volume(shp); 
                % Compute volume
        tempdata = double(full(TR_template.Points));
        shp = alphaShape(tempdata);
        targetvol = volume(shp); 
                % Compute volume
        scalefactor = inputvol/targetvol;
        a = 2;
    elseif bone_indx == 13 && bone_coord == 1
        TR_template = stlread('Tibia_Template.stl');
        a = 3;
    elseif bone_indx == 13 && bone_coord == 2
        TR_template = stlread('Tibia_Template_Facet.stl');
        a = 3;
    elseif bone_indx == 14 && bone_coord == 1
        TR_template = stlread('Fibula_Template.stl');
        a = 3;
    elseif bone_indx == 14 && bone_coord == 2
        TR_template = stlread('Fibula_Template_Facet.stl');
        a = 3;
    end
    target = TR_template.Points;
    target = target*scalefactor; % Scale to match data
    % Sample data if required
    if ~isempty(opts.sample)
        if size(data, 1) || size(target,1) <= opts.sample
            %%determine which is larger, data or target
            if size(data, 1) > size(target, 1)
                D = sampleData(data, size(target, 1));
                T = sampleData(target, size(target, 1));
            else
                T = sampleData(target, size(data, 1));
                D = sampleData(data, size(data, 1));
            end
        else
            D = sampleData(data, opts.sample);
            T = sampleData(target, opts.sample);
        end
    else
        D = data;
        T = target;
    end
    % fprintf('Sampled data size: %d\n', size(D, 1));
    % fprintf('Sampled target size: %d\n', size(T, 1));
    % Initial guess
    if isempty(opts.t0)
        t0 = zeros(1,6);
    else
        t0 = opts.t0(:)';
    end
    
    % Rotation center
    if isempty(opts.rotcentre)
        rotcentre = mean(D, 1);
    else
        rotcentre = opts.rotcentre;
    end
    
    % Objective function
    if size(data,1) >= numel(t0)
        % Return residuals as double vector
        obj = @(x) double(vecnorm(transformRigid3DAboutP(D, x, rotcentre) - T, 2, 2));
    else
        % Return scalar for fminsearch
        obj = @(x) double(sum(vecnorm(transformRigid3DAboutP(D, x, rotcentre) - T, 2, 2).^2));
    end
    
    rms0 = sqrt(mean(obj(t0)));
    
    if opts.verbose
        fprintf('Initial RMS: %.6f\n', rms0);
    end
    
    % Optimization
    if size(data,1) >= numel(t0)
        if isempty(opts.maxfev)
            opts.maxfev = 0;
        end
        options = optimset('TolX', opts.xtol, 'MaxFunEvals', opts.maxfev, 'Display', 'off');
        [x_opt,resnorm,residual,exitflag,output] = lsqnonlin(obj, t0, [], [], options);
    else
        options = optimset('TolX', opts.xtol, 'MaxIter', opts.maxfev, ...
                           'MaxFunEvals', opts.maxfun, 'Display', logical2disp(opts.verbose));
        x_opt = fminsearch(obj, t0, options);
    end
    
    rms_opt = sqrt(mean(obj(x_opt)));
    
    if opts.verbose
        fprintf('Final RMS: %.6f\n', rms_opt);
        fprintf('Exit flag: %d\n', exitflag);
    end
    
    data_fitted = transformRigid3DAboutP(data, x_opt, rotcentre);
    
    % Output
    if opts.output_errors
        rms_vals = [rms0, rms_opt];
    else
        rms_vals = [];
    end
    
    end
    
    function dispStr = logical2disp(flag)
        if flag
            dispStr = 'iter';
        else
            dispStr = 'off';
        end
    end
    