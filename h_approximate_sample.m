%approximate the h_value of the multi-independent poisson distribution 
function H_appro = h_approximate_sample(set, path, triple, tau, presamp_effi, risk_level, n_samp)
              %note that set is a taxi set s_i for demand location id
              
              %note that pair is [i, j] pair each location i and robot j
              %set is a N*1 cell, contains the assignemnt for each location
              %i.            
              % the submodular funcion is f(S,y): = sum_i max_j\in S_i
              % e_ij.
              
              % update the set 
              set{triple(1)} = [set{triple(1)}, triple(2)];
              
              %update the path
              path{triple(1)} = [path{triple(1)}, triple(3)];
              
              sum_max_demand = efficiency_distribution(set, path, presamp_effi, n_samp);        
                                                      
              tail_h = max((tau * ones(1, n_samp) - sum_max_demand), zeros(1, n_samp)); 
              
              H_appro = tau - (1/risk_level) * mean(tail_h); 

end