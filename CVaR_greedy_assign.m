% cvar greedy for multi-robot assignment
function [cvar_greset, cvar_grepath, cvar_gre_distribution, cvar_grehvalue, cvar_gre_add,...
    cvar_gre_tau,cvar_gre_curv, tau_hvalue, tau_hstar, alltau_greset, max_hstar_bound]...
    = CVaR_greedy_assign(effi_presamps, alpha, serh_sep, n_samp, upper_bound)

    global n_goal n_robot n_path
        
    % the # of tau(s) for searching, 
    % tau is the second variable in the auxiliary function h(S,tau).
    n_tau = upper_bound/serh_sep; 
    
    % store the greedy curvature for each tau, [tau, tau_curv]
    tau_gre_hcurv = zeros(n_tau, 2);
    
    % store tau and h_value, [tau, h_value]
    tau_hvalue = zeros(n_tau, 2);
   
    % store the upper bound of h as h* 
    tau_hstar = zeros(n_tau, 1); 
    
    % store the set selected by greedy at each tau
    % its structure: the set of start positions to each goal [S_1; S_2; S_3.....]
    alltau_greset = cell(n_tau, 1); 
    
    % store the corresponding path selected for the greset at each tau.
    alltau_grepath = cell(n_tau,1);
    
    % a counter for the # of tau(s)
    cnt = 1;
    for tau = serh_sep : serh_sep : upper_bound       
        % since we need to assign a set of start positions S_i to each goal
        % position i. we need to keep each S_i separately. 
         
        %/*** Greedy Algorithm ***/ 
        % create gre_set to store S_i (s) at each tau 
        greset_Si_eachtau = cell(n_goal, 1);
        
        % store the corresponding selected path for S_i in gre_set at each tau.
        grepath_Si_eachtau = cell(n_goal, 1);
        
        % keep track of the start positions that have been assigned,
        % since it can be assigned at most once.
        greset_assigned_eachtau = [];
        
        %the greedy curvature at each tau
        grecurv_eachtau = [];
        
        % we need to initialize the previous greedy values
        gre_last_hvalue_eachtau  = tau * (1 - 1/alpha); 
        
        %n_start rounds, we need to assign all the start positions
         for r  = 1 : n_robot
              %keep margin gain at each round for each start position i 
              %store start pos i, margin_gain, current_hvalue at each round
              %store inx of ij, margin_gain, h_value current at each round
              %of selection
              % this step we calucalte H(SU{i, j}) - H(S), we would like to 
              % separate this into N rows, and R - length(gre_selected)
              % columns
              
              start_goal_inx = zeros(n_goal, n_robot - length(greset_assigned_eachtau)); 
              path_inx = zeros(n_goal, n_robot- length(greset_assigned_eachtau)); 
              
              margin_gain_ij = zeros (n_goal, n_robot-  length(greset_assigned_eachtau)); 
              hvalue_current_ij = zeros(n_goal, n_robot - length(greset_assigned_eachtau)); 
              
              curv = zeros(n_goal, n_robot - length(greset_assigned_eachtau)); 
             
              % n_goal positions
              for i  =  1 : n_goal
                  %n_start positions
                  cnt_j = 1; 
                  for  j  = 1 : n_robot  
                      % first check if start pos j has not been selected,
                      % if not, check paths. 
                      if ismember(j, greset_assigned_eachtau) == 0 
                          % we need to check n_paths from start j to goal i
                          
                          margin_gain_ijk = zeros(1,n_path);
                          hvalue_current_ijk = zeros(1, n_path);
                          for k = 1 : n_path
                              gre_current_hvalue_ijk = h_approximate_sample(greset_Si_eachtau, grepath_Si_eachtau, [i,j,k], ...
                                  tau, effi_presamps, alpha, n_samp); 
                              margin_hvalue_ijk =  gre_current_hvalue_ijk - gre_last_hvalue_eachtau; 
                              
                              % store margin_gain for kth path from start j
                              % to goal i. 
                              margin_gain_ijk(k) = margin_hvalue_ijk; 
                              hvalue_current_ijk(k) = gre_current_hvalue_ijk; 
                          end        
                          % find which path has the maximum marginal gain
                          % keep it.
                          [margin_hvalue_ij, max_path_inx] = max(margin_gain_ijk);
                          
                          %keep the path inx in the table
                          path_inx(i, cnt_j) = max_path_inx; 
                          
                          %keep the start position inx in the table 
                          start_goal_inx(i, cnt_j) = j; 
                          
                          %keep the margin gain in the table
                          margin_gain_ij(i, cnt_j) = margin_hvalue_ij; 
                          
                          %keep the h_current value in the table
                          hvalue_current_ij(i, cnt_j) = hvalue_current_ijk(max_path_inx); 
                          
                          % keep the curvature in the table
                          curv(i, cnt_j) = 1 - ...
                          margin_hvalue_ij/(hvalue_current_ijk(max_path_inx) - tau * (1 - 1/alpha));
                         
                          cnt_j = cnt_j + 1; 
                      else % skip j

                      end
                  end %the end of n_start positions
              end% the end of n_goal positions for S_i
              
              % collect all the marginal gains   
              % find the maximum margin gain in the talble, 
              % remember the inx in the table is not the real inx of i,j
               [row_inx, col_inx] = find(margin_gain_ij == max(max(margin_gain_ij)), 1); 
               
               % Then we know, the row_inx is the goal, 
               % col_inx is the inx in the table that corresponds a start
               % position j. 
               
               % update the greset_Si_eachtau, S_i the set of start
               % positions assigned to goal i.
               greset_Si_eachtau{row_inx} = [greset_Si_eachtau{row_inx}, start_goal_inx(row_inx, col_inx)];
               
               % update the corresponding path inx for S_i
               grepath_Si_eachtau{row_inx} = [grepath_Si_eachtau{row_inx}, path_inx(row_inx, col_inx)]; 
               
               % update the greset_assigned_eachtau, store all start
               % positions have been assigned
               greset_assigned_eachtau = [greset_assigned_eachtau, start_goal_inx(row_inx, col_inx)];
                             
               %store greedy curvature at each round
               grecurv_eachtau = [grecurv_eachtau, max(curv(:))]; 
              
               %updatethe gre_hvalue_last, greedy iteration     
               gre_last_hvalue_eachtau = hvalue_current_ij(row_inx, col_inx); 
                            
         end % the end of R round for greedy algorithm    
         
         %store s_i(s) inside it
         alltau_greset{cnt} = greset_Si_eachtau;
         
         % store gre_path_eachtau 
         alltau_grepath{cnt} = grepath_Si_eachtau; 
         
         % store greedy curvature
         tau_gre_hcurv(cnt, :) = [tau, max(grecurv_eachtau)]; 
         
         %store tau_hvalue
         tau_hvalue(cnt, :) = [tau, gre_last_hvalue_eachtau]; 
         
         %calculate the H* value in the partition case
         tau_hstar(cnt) = gre_last_hvalue_eachtau; %+ (1/2)* tau * (1/alpha -1);  
                
         cnt = cnt + 1; 
    end
    % tau_loop ends    
    
       % find the maximum tau_hstar
       [max_hstar_bound, max_hstar_inx] = max(tau_hstar); 
   
       % find the associated set we choose, this is a set. 
       cvar_greset = alltau_greset{max_hstar_inx}; 
       
       % find the associated path
       cvar_grepath = alltau_grepath{max_hstar_inx};
   
       % find the associated cvar_gre_value by the greedy approach
       cvar_grehvalue = tau_hvalue(max_hstar_inx,2);
       
       %calculate the additivity term
       cvar_gre_tau = tau_gre_hcurv(max_hstar_inx, 1);
       cvar_gre_curv =  tau_gre_hcurv(max_hstar_inx, 2); 
       cvar_gre_add = cvar_gre_tau*(1/alpha - 1) * ...
           cvar_gre_curv/(1 + cvar_gre_curv);        
       %calculate the uncertainty, we know that the mean and the
       %uncertainty for poisson distribution are the same. 
       [cvar_gre_distribution] = efficiency_distribution(cvar_greset, cvar_grepath, effi_presamps, n_samp);
       %[~, cvar_gre_distribution] = efficiency_distribution_samp(cvar_gre_set, efficiency, tau, n_s);
end