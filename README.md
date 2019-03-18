# risk_aware-_path_assignment

main function: start_goal_path_assign.m

"cvar_greset_rlevels" stores the robots assign to goal 1, 2, ... with different risk levels, 0.01, 0.1, ..., 1

For example, [2; 1, 3] denotes robot 2 is assigned to goal 1 and robots 1 and 3 are assigned to goal 2

"cvar_grepath_rlevels" stores the corresponding paths of the assigned robots [2; 1, 3]. 

[1; 1, 2] denotes robot 2's path 1,  robot 1's path 1 and robot 3's path 2 are chosen. 

