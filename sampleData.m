function sampled = sampleData(data, n_points)
    % Pick N evenly spaced points from data
    
    if n_points < 1
        error('n_points must be greater than 0');
    elseif n_points > size(data, 1)
        sampled = data;
    else
        indices = round(linspace(1, size(data, 1), n_points));
        sampled = data(indices, :);
    end
    end
    