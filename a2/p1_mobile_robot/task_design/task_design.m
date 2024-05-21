% task_design: Function for specifying the task to fulfilled by the mobile
%              robot as well as the parameters for simulation.
%
% Output:
%       task:	A structure that describes the task to be
%               fulfilled/simulated. This includes parameters of the cost
%               function, initial and final time, sampling interval,
%               initial and goal states.
%
% --
% Control for Robotics
% Assignment 2
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
% [20.01.31, SZ]    first version

function task = task_design( )
    % defines a task for a controller to execute
    task = struct;

    % intial and final simulation time, and discrete time step
    task.start_time = 0;	% start time
    task.end_time = 15;     % end time
    task.dt = 0.02;         % discrete time step

    % start and final states
    task.start_x = [10; 0];   % [y; h]
    task.goal_x = [-5; 0];   % [y; h]
    
    % stage cost function parameters
    task.cost.params.Q_s = diag([1, 1]);
    task.cost.params.R_s = 20;
    
    % terminal cost function parameters
	task.cost.params.Q_t = diag([1, 1]);
    
    % maximum ILQC iterations
    task.max_iteration  = 15;
end