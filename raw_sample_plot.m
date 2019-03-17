% distribution plot
% Main function for risk-averse path assignment. 
% It contains two parts: presamples and cvar optimization
cla; clear all;

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

% presample 
[effi_presamps_path, cost_presamps_path] = presample(pixel_cost_sample, all_paths, n_samp);

% effi is a small number say 10^(-4), here I enlarge it by 10^5 
effi_presamps_path = effi_presamps_path * 10^5;


xaxis_left = min(effi_presamps_path(:));
xaxis_right = max(effi_presamps_path(:));


figure (11), hold on % goal 1 robot 1 three paths
s(1) = subplot(2,1,1);
histogram(s(1), effi_presamps_path(1,1,1,:)), hold on
axis([xaxis_left xaxis_right 0 inf])
title('Goal 1, Robot 1, Paths 1', 'fontsize', 14)

s(2) = subplot(2,1,2);
histogram(s(2), effi_presamps_path(1,1,2,:)), hold on
axis([xaxis_left xaxis_right 0 inf])
title('Goal 1, Robot 1, Paths 2', 'fontsize', 14)

% s(3) = subplot(3,1,3);
% histogram(s(3), effi_presamps_path(1,1,3,:))
% axis([xaxis_left xaxis_right 0 inf])
% title('Goal 1, Robot 1, Paths 3', 'fontsize', 14)



figure (12), hold on % goal 1 robot 1 three paths
s(1) = subplot(2,1,1);
histogram(s(1), effi_presamps_path(1,2,1,:)), hold on
axis([xaxis_left xaxis_right 0 inf])
title('Goal 1, Robot 2, Paths 1', 'fontsize', 14)

s(2) = subplot(2,1,2);
histogram(s(2), effi_presamps_path(1,2,2,:)), hold on
axis([xaxis_left xaxis_right 0 inf])
title('Goal 1, Robot 2, Paths 2', 'fontsize', 14)

% s(3) = subplot(3,1,3);
% histogram(s(3), effi_presamps_path(1,2,3,:))
% axis([xaxis_left xaxis_right 0 inf])
% title('Goal 1, Robot 2, Paths 3', 'fontsize', 14)



figure (13), hold on % goal 1 robot 1 three paths
s(1) = subplot(2,1,1);
histogram(s(1), effi_presamps_path(1,3,1,:)), hold on
axis([xaxis_left xaxis_right 0 inf])
title('Goal 1, Robot 3, Paths 1', 'fontsize', 14)

s(2) = subplot(2,1,2);
histogram(s(2), effi_presamps_path(1,3,2,:)), hold on
axis([xaxis_left xaxis_right 0 inf])
title('Goal 1, Robot 3, Paths 2', 'fontsize', 14)

% s(3) = subplot(3,1,3);
% histogram(s(3), effi_presamps_path(1,3,3,:))
% axis([xaxis_left xaxis_right 0 inf])
% title('Goal 1, Robot 3, Paths 3', 'fontsize', 14)



figure (21), hold on % goal 1 robot 1 three paths
s(1) = subplot(2,1,1);
histogram(s(1), effi_presamps_path(2,1,1,:)), hold on
axis([xaxis_left xaxis_right 0 inf])
title('Goal 2, Robot 1, Paths 1', 'fontsize', 14)

s(2) = subplot(2,1,2);
histogram(s(2), effi_presamps_path(2,1,2,:)), hold on
axis([xaxis_left xaxis_right 0 inf])
title('Goal 2, Robot 1, Paths 2', 'fontsize', 14)

% s(3) = subplot(3,1,3);
% histogram(s(3), effi_presamps_path(2,1,3,:))
% axis([xaxis_left xaxis_right 0 inf])
% title('Goal 2, Robot 1, Paths 3', 'fontsize', 14)



figure (22), hold on % goal 1 robot 1 three paths
s(1) = subplot(2,1,1);
histogram(s(1), effi_presamps_path(2,2,1,:)), hold on
axis([xaxis_left xaxis_right 0 inf])
title('Goal 2, Robot 2, Paths 1', 'fontsize', 14)

s(2) = subplot(2,1,2);
histogram(s(2), effi_presamps_path(2,2,2,:)), hold on
axis([xaxis_left xaxis_right 0 inf])
title('Goal 2, Robot 2, Paths 2', 'fontsize', 14)

% s(3) = subplot(3,1,3);
% histogram(s(3), effi_presamps_path(2,2,3,:))
% axis([xaxis_left xaxis_right 0 inf])
% title('Goal 2, Robot 2, Paths 3', 'fontsize', 14)



figure (23), hold on % goal 1 robot 1 three paths
s(1) = subplot(2,1,1);
histogram(s(1), effi_presamps_path(2,3,1,:)), hold on
axis([xaxis_left xaxis_right 0 inf])
title('Goal 2, Robot 3, Paths 1', 'fontsize', 14)

s(2) = subplot(2,1,2);
histogram(s(2), effi_presamps_path(2,3,2,:)), hold on
axis([xaxis_left xaxis_right 0 inf])
title('Goal 2, Robot 3, Paths 2', 'fontsize', 14)

% s(3) = subplot(3,1,3);
% histogram(s(3), effi_presamps_path(2,3,3,:))
% axis([xaxis_left xaxis_right 0 inf])
% title('Goal 2, Robot 3, Paths 3', 'fontsize', 14)


% %% Gaussion test
% % create two tested gaussian distributions
% n_samp = 5000;
% effi_presamps_path = zeros(n_goal, n_robot, n_path, n_samp);
% path1 = [];
% path2 = [];
% while length(path1) <= n_samp
%     r1 = normrnd(50,10); 
%     if r1>=0 && r1<=100
%         path1 = [path1; r1]; 
%     end
% 
% end
% effi_presamps_path(1,1,1,:) = path1(1:n_samp,1);
% while length(path2) <= n_samp
%         r2 = normrnd(40,5);
%         if r2>=30 && r2 <=60
%          path2 = [path2; r2]; 
%         end
% end
% effi_presamps_path(1,1,2,:) = path2(1:n_samp,1);
% figure (1)
% s(1) = subplot(2,1,1);
% s(2) = subplot(2,1,2);
% histogram(s(1), effi_presamps_path(1,1,1,:)), hold on
% histogram(s(2), effi_presamps_path(1,1,2,:))
% upper_bound = n_goal * round(max(effi_presamps_path(:)));