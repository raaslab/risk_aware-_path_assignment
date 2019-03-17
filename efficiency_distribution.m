% calculate the mean and the uncertainty after obtainting the set 
function  sum_max_goal = efficiency_distribution(set, path, presamp_effi, n_s)

        % return the distribution of the f(S,y) = \sum_i max_j e_ij
        % search for all goal locations
        max_each_goal = zeros(length(set), n_s); 
        
        sum_max_goal = zeros(1, n_s);
        
        for i = 1 : length(set)
                  %for each goal location, calculate the maximum vector
                  if isempty(set{i}) == 0 % no empty
                      
                      for j = 1 : length(set{i})
                           
                           max_each_goal(i, :) = max(max_each_goal(i, :), ...
                               reshape(presamp_effi(i, set{i}(j), path{i}(j),:),[1, n_s]));                      
                      end
                      
                  end
                  
                  sum_max_goal = sum_max_goal + max_each_goal(i, :);                         
        end           
end