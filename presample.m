% presample the efficiency, which is a random variable
function [effi_presamps_path, cost_presamps_path] = presample(pixel_cost_sample, paths, n_samp)

    global n_robot n_goal n_path
    [size_x, ~, ~] = size(pixel_cost_sample);
    
    % % the pixel coordination on each path
    path_pixels_coord = cell(n_goal, n_robot, n_path);

    %funtion: pre-sample from the efficiency.
    %presamp_effi is a 4 dimension arrary: [n_start, n_goal, n_paths, n_samp]
    cost_presamps_path = zeros(n_goal, n_robot, n_path, n_samp);
    % get the data and samples from paths 
    for i = 1 : n_goal
        for j = 1 : n_robot
            for k = 1 : n_path
%                 path_pixels_coord{i, j, k} = paths{1, 3*n_path + 2*n_path + k};
                 path_pixels_coord{i, j, k} = paths{1, (i-1)*n_robot*n_path + (j-1)*n_path + k};
                % for each sample of the cost on the path.
                for h = 1 : n_samp
                    temp = 0;
                    for l = 1 : length(path_pixels_coord{i, j, k}(:,1))
                         temp   = temp ...
                      + pixel_cost_sample(randi(size_x, 1, 1,'int8'), ...
                                             path_pixels_coord{i, j, k}(l,2), ...
                                             path_pixels_coord{i, j, k}(l,1));
                    end
                    cost_presamps_path(i,j,k,h) = temp; 
                    
                end
            end
        end
    end
    
    % take the reciprocal of the pixel_cost to get the efficiency 
    effi_presamps_path = 1./cost_presamps_path;
end
