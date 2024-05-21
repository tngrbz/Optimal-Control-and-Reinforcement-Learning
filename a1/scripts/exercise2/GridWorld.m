% GridWorld: Grid world class for Problem 1.2 Dynamic Programming for a 
%            Robot Vacuum Cleaner.
%
% --
% Control for Robotics
% AER1517 Spring 2022
% Assignment 1
%
% --
% University of Toronto Institute for Aerospace Studies
% Dynamic Systems Lab
%
% Course Instructor:
% Angela Schoellig
% schoellig@utias.utoronto.ca
%
% Teaching Assistants: 
% SiQi Zhou: siqi.zhou@robotics.utias.utoronto.ca
% Adam Hall: adam.hall@robotics.utias.utoronto.ca
% Lukas Brunke: lukas.brunke@robotics.utias.utoronto.ca
%
% --
% Revision history
% [22.01.17, LB]    first version
%
% The grid world is the set up in the following way:
% - - - - - - -
% | d d o o c |
% | d x o o o |
% | d d o o s |
% | o o o o o |
% - - - - - - -
%
% Legend
% d: dirt
% x: obstacle
% o: empty cell 
% c: charger
% s: carpet
%
% Actions
% Charge: 0
% North:  1
% East:   2
% South:  3
% West:   4
%

classdef GridWorld
   
    properties 
        % set up grid
        num_rows = 4;
        num_columns = 5;
        
        % define costs
        cost_charger = 0;
        cost_obstacle = inf;
        cost_empty_cell = 6;
        cost_dirt = 5;
        cost_carpet = 100;
        
        % initialize stage cost
        stage_cost = 0;
        obstacle_pos = 0;
        charger_pos = 0;
        stage_cost_assigned = false;
    end
    
    methods        
        % constructor of the grid world with initialized stage cost
        function obj = GridWorld()
            % assign cost to cells
            obj.stage_cost = obj.cost_empty_cell * ones(obj.num_rows, obj.num_columns);
            obj.stage_cost(1, 1) = obj.cost_dirt;
            obj.stage_cost(1, 2) = obj.cost_dirt;
            obj.stage_cost(1, 5) = obj.cost_charger;
            obj.stage_cost(2, 1) = obj.cost_dirt;
            obj.stage_cost(2, 2) = obj.cost_obstacle;
            obj.stage_cost(3, 1) = obj.cost_dirt;
            obj.stage_cost(3, 2) = obj.cost_dirt;
            obj.stage_cost(3, 5) = obj.cost_carpet; 
            
            obj.obstacle_pos = [2; 2];
            obj.charger_pos = [1; 5];
            
            obj.stage_cost_assigned = true;
        end
        % lists the available actions at state x
        function actions = available_actions(obj, x)
            actions = [];
            % make sure the cell is part of the gridworld
            if x(1) > 0 && x(1) <= obj.num_rows && x(2) > 0 && ...
                    x(2) <= obj.num_columns
                % the robot is only allowed to stay at the charger
                if all(x == obj.charger_pos)
                    actions = [actions, 0];
                end
                % determine the states where the robot can go NORTH
                if x(1) > 1 && ~all(x == obj.obstacle_pos) ...
                        && ~all(x == obj.obstacle_pos + [1; 0])
                    actions = [actions, 1];
                end
                % determine the states where the robot can go EAST
                if x(2) < obj.num_columns && ~all(x == obj.obstacle_pos) ...
                        && ~all(x == obj.obstacle_pos + [0; -1])
                    actions = [actions, 2];
                end
                % determine the states where the robot can go SOUTH
                if x(1) < obj.num_rows && ~all(x == obj.obstacle_pos) ... 
                        && ~all(x == obj.obstacle_pos + [-1; 0])
                    actions = [actions, 3];
                end
                % determine the states where the robot can go WEST
                if x(2) > 1 && ~all(x == obj.obstacle_pos) ...
                        && ~all(x == obj.obstacle_pos + [0; 1])
                    actions = [actions, 4];
                end
            end
        end
        % indicates in which cells an action a in [0, 1, 2, 3, 4] is
        % available
        function action_maps(obj, a)
            action_map = zeros(obj.num_rows, obj.num_columns);
            for i = 1 : obj.num_rows
                for j = 1 : obj.num_columns
                    actions = obj.available_actions([i; j]);
                    if ismember(a, actions)
                        action_map(i, j) = 1;
                    end
                end
            end
            action_map
        end
        % determine the next state after applying action a at state x
        function x_next = next_state(obj, x, a)
            if ~ismember(a, obj.available_actions(x))
                disp("Action is not available.")
                x_next = x;
                return
            end
            if a == 0
                x_next = x;
            elseif a == 1
                x_next = x + [-1; 0];
            elseif a == 2
                x_next = x + [0; 1];
            elseif a == 3 
                x_next = x + [1; 0];
            elseif a == 4
                x_next = x + [0; -1];
            end
        
        end
        % plots the sequence of states after applying a sequence of actions
        function plot_moves(obj, x, actions)
            map = zeros(obj.num_rows, obj.num_columns);
            map(x(1), x(2)) = 1;
            for i = 1 : length(actions)
                x = obj.next_state(x, actions(i));
                map(x(1), x(2)) = i + 1;
            end
            map
        end
        
    end
    
end