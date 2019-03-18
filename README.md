# risk_aware-_path_assignment

main function: start_goal_path_assign.m

Run the codes before Figure plot (line 158)

"cvar_greset_rlevels" stores the robots assign to goal 1, 2, ... with different risk levels,

For example, [2; 1, 3] denotes robot 2 is assigned to goal 1 and robots 1 and 3 are assigned to goal 2

"cvar_grepath_rlevels" stores the corresponding paths of the assigned robots. 
[1; 1, 2] denotes robot 2's path 1,  robot 1's path 1 and robot 3's path 2 are chosen. 

The codes from line 69-114 are testing codes for 1 goal 1 robot and 3 paths with Gaussian distributions
you may need to set 
n_goal = 1;
n_robot = 1;
n_path = 3; 
for testing. 
