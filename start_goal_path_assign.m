% Main function for risk-averse path assignment. 
% It contains two parts: presamples and cvar optimization
clear a

global n_goal n_robot n_path

% the number of goal_positions, 
% we need to assign a set of start positions to each goal position
n_goal = 2;

% the number of start_positions
n_robot = 3;

% the number of paths from each start to each goal 
n_path = 2; 

%/***First part: Pre-Sample of the efficiency***/
% the number of samples for each efficiency
n_samp = 2000; 

%load costs
load('classes_cost_v5.mat'); 
%load coordinations of the paths
load('00009_paths_cell_v5.mat')

% define a matrix to receive all pixel_samples
pixel_cost_sample = classes_cost;

% the pixel coordinations in all paths 
all_paths = paths;

% % plot paths
% figure (1)
% axis([0 360 0 480])
% 
% for i = 16 : 18
%     plot(all_paths{i}(:,1), all_paths{i}(:,2)); hold on
% end

% presample 
[effi_presamps_path, cost_presamps_path] = presample(pixel_cost_sample, all_paths, n_samp);


%/***Second part: CVaR Optimization***/
effi_presamps_path = effi_presamps_path * 10^5;

% xaxis_left = min(effi_presamps_path(:));
% xaxis_right = max(effi_presamps_path(:));
% the upper bound for tau, because there are n_goal positions
% we need to sum up the n_goal positions
upper_bound = n_goal * round(max(effi_presamps_path(:)));


% figure (1)
% s(1) = subplot(3,1,1);
% histogram(s(1), effi_presamps_path(1,1,1,:)), hold on
% axis([xaxis_left xaxis_right 0 inf])
% title('Goal 1, Robot 1, Paths 1', 'fontsize', 14)
% 
% s(2) = subplot(3,1,2);
% histogram(s(2), effi_presamps_path(1,1,2,:)), hold on
% axis([xaxis_left xaxis_right 0 inf])
% title('Goal 1, Robot 1, Paths 2', 'fontsize', 14)
% 
% s(3) = subplot(3,1,3);
% histogram(s(3), effi_presamps_path(1,1,3,:))
% axis([xaxis_left xaxis_right 0 inf])
% title('Goal 1, Robot 1, Paths 3', 'fontsize', 14)
% %%
% % create two tested gaussian distributions
% n_samp = 5000;
% effi_presamps_path = zeros(n_goal, n_robot, n_path, n_samp);
% % path1 = [];
% % path2 = [];
% % while length(path1) <= n_samp
% %     r1 = normrnd(50,10); 
% %     if r1>=0 && r1<=100
% %         path1 = [path1; r1]; 
% %     end
% % 
% % end
% path1 = normrnd(50,10, n_samp, 1); 
% effi_presamps_path(1,1,1,:) = path1(1:n_samp,1);
% % while length(path2) <= n_samp
% %         r2 = normrnd(40,5);
% %         if r2>=30 && r2 <=60
% %          path2 = [path2; r2]; 
% %         end
% % end
% path2 = normrnd(40,4, n_samp, 1); 
% effi_presamps_path(1,1,2,:) = path2(1:n_samp,1);
% 
% path3 = normrnd(35,2, n_samp, 1); 
% effi_presamps_path(1,1,3,:) = path3(1:n_samp,1);
% 
% xaxis_left = min(effi_presamps_path(:));
% xaxis_right = max(effi_presamps_path(:));
% 
% figure (1)
% 
% s(1) = subplot(3,1,1);
% histogram(s(1), effi_presamps_path(1,1,1,:)), hold on
% axis([xaxis_left xaxis_right 0 inf])
% title('Paths 1', 'fontsize', 14)
% 
% s(2) = subplot(3,1,2);
% histogram(s(2), effi_presamps_path(1,1,2,:)), hold on
% axis([xaxis_left xaxis_right 0 inf])
% title('Paths 2', 'fontsize', 14)
% 
% s(3) = subplot(3,1,3);
% histogram(s(3), effi_presamps_path(1,1,3,:)), hold on
% axis([xaxis_left xaxis_right 0 inf])
% title('Paths 3', 'fontsize', 14)
% 
% upper_bound = n_goal * round(max(effi_presamps_path(:)));
%%
% user-defined searching separation for tau
serh_sep = 0.5;
% user-defined risk levels
risk_levels = [0.01, 0.05, 0.1, 0.5, 0.8, 1.0];
%risk_levels = 0.01; 

%define the storage space for all risk levels

% h(s^{g}, tau)
cvar_hvalue_rlevels = zeros(length(risk_levels),2); 
% the additive error term
cvar_adderr_rlevels = zeros(length(risk_levels),2);
% the distribution
cvar_gredis_rlevels = cell(length(risk_levels),1); 
% the set selected by the greedy
cvar_greset_rlevels = cell(length(risk_levels),1); 
% the corresponding path
cvar_grepath_rlevels = cell(length(risk_levels),1); 

% for each risk level
for i = 1 : length(risk_levels)
    
    risk_level = risk_levels(i); 
    
    % CVaR greedy assignment
    [cvar_greset, cvar_grepath, cvar_gredistri, cvar_grehvalue, cvar_greadd,...
        cvar_gre_tau, cvar_gre_curv, tau_hvalue, tau_hstar, alltau_greset, max_hstar_bound]...
        = CVaR_greedy_assign(effi_presamps_path, risk_level, serh_sep, n_samp, upper_bound); 
    
    % store [tau, h]
    cvar_hvalue_rlevels(i, :) = [risk_level, cvar_grehvalue];
    % store [tau, adderr]
    cvar_adderr_rlevels(i, :) = [risk_level, cvar_greadd];
    % store {cvar_gredis}
    cvar_gredis_rlevels{i} = cvar_gredistri;
    % store {cvar_greset}
    cvar_greset_rlevels{i} = cvar_greset;
    % store {cvar_grepath}
    cvar_grepath_rlevels{i} = cvar_grepath;
end
%%
% plot h(S^{gre},tau) w.r.t. risk levels
figure (1)
plot(cvar_hvalue_rlevels(:,1), cvar_hvalue_rlevels(:,2), 'r*'), hold on 

% plot additive error w.r.t. risk levels
figure (2)
plot(cvar_adderr_rlevels(:,1), cvar_adderr_rlevels(:,2), 'bo'), hold on

% plot distribution of f(S,y) w.r.t. risk levels
figure (3)
nhist(cvar_gredis_rlevels, 'legend', {'$$\alpha=0.001$$', '$$\alpha=0.01$$', ...
 '$$\alpha=0.01$$', '$$\alpha=0.3$$', ...
 '$$\alpha=0.6$$', '$$\alpha=0.9$$', '$$\alpha=1$$'}); 

%%
% we use greey set and greey path to illustrate the simple path assignment examples.

% plot goals, starts, paths
figure (4)
plot(goals(:,1), goals(:,2), 'rx'), hold on
plot(starts(:,1), starts(:,2), 'ks'), hold on
plot(paths)

% when risk_level = 0.1
cvar_greset_rlevels{3};
cvar_grepath_rlevels{3};

for i = 1 : length(cvar_greset_rlevels{3}) % search goals
    for j = 1 : length(cvar_greset_rlevels{3}{i}) % for each goal i, evaluate S_i
        % the goal pos [x,y]
        [goals(i,1), goals(i,2)];
        % the start pos assigned [x,y]
        [starts(cvar_greset_rlevels{3}{i}(j,1)), starts(cvar_greset_rlevels{3}{i}(j,2))]
        % the path selected 
        cvar_grepath_rlevels{3}{i}(j);
    end
end