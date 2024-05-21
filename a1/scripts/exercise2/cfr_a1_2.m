% cfr_a1_2: Main script for Problem 1.2 Dynamic Programming for a 
%               Robot Vacuum Cleaner.
%
% --
% Control for Robotics
% Summer 2023
% Assignment 1
%
% --
% Technical University of Munich
% Learning Systems and Robotics Lab
%
% Course Instructor:
% Angela Schoellig
% schoellig@utias.utoronto.ca
%
% Teaching Assistants: 
% SiQi Zhou: siqi.zhou@tum.de
% Lukas Brunke: lukas.brunke@tum.de
%
% --
% Revision history
% [22.01.17, LB]    first version
% [22.01.23, LB]    added 2 (c) to the code, removed N

clear all
close all
clc

%% calculate optimal control using dynamic programming

% initialize the grid world
grid = GridWorld();

% allocate arrays for optimal control inputs and cost-to-go 
U = zeros(grid.num_rows, grid.num_columns);
J = zeros(grid.num_rows, grid.num_columns);

% set the cost for the obstacle
J(grid.obstacle_pos(1), grid.obstacle_pos(2)) = inf;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO: YOUR CODE HERE - Exercise 2 (a)
J_log = ~logical(J); 
% This logical matrix is there to be able to change the cost function J
% because the changing condition see line 63 doesn't work at the beginning
% since all costs are 0.

x_last = grid.charger_pos;
J_log(x_last(1), x_last(2)) = false; 
%false beacuse I dont want it to be changed.

states_to_visit = x_last;
k = 0;
while k<100
    if x_last == [1;5]
        av_ac = [3, 4];
    else
        av_ac = grid.available_actions(x_last);
    end
    % I want to find a path from end to beginning and 
    % since the available action is 0 for the charger state,
    % I created a artificial path.

    for i = 1:length(av_ac)
        next_state = grid.next_state(x_last, av_ac(i));
        x = next_state(1);
        y = next_state(2);
        cost = J(x_last(1), x_last(2)) + grid.stage_cost(x, y);
        if (cost < J(x, y) || J_log(x, y) == true)
            J_log(x, y) = false;
            J(x, y) = cost;
            U(x, y) = av_ac(i);
            states_to_visit = [states_to_visit, next_state];
        end
    end
    if size(states_to_visit, 2) <= 1
        break
    end
    states_to_visit = states_to_visit(:,2:end);
    x_last = states_to_visit(:,1);
    k = k + 1;
end
J
% We started from end towards to beginning so I change the directions of
% the actions.
mask = find(U == 2);
U = mod(U+2, 4);
U(mask) = 4;
U

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Simulate robot vacuum cleaner
x_0 = [4; 3];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO: YOUR CODE HERE - Exercise 2 (b)
optimal_actions = [];
while true
    if x_0 == grid.charger_pos
        break
    end
    act = U(x_0(1), x_0(2));
    optimal_actions = [optimal_actions, act];
    x_next = grid.next_state(x_0, act);
    x_0 = x_next;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x_0 = [4; 3];
grid.plot_moves(x_0, optimal_actions)

%% Simulate robot vacuum cleaner
x_0 = [4; 3];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO: YOUR CODE HERE - Exercise 2 (c)
% Changed the cost manually in Gridworld.m file and run the script from the
% beginning 



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

grid.plot_moves(x_0, optimal_actions)
