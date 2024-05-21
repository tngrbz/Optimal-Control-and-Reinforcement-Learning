% mobile_robot_sim_test: Function for simulating the system response given
%               a sequence of controller parameters for k = 1, 2,..., N-1.  
%
% Note: This function calls the modified model 'unicycle_test' to simulate
%       the mismatch between the system model and the actual system
%       dynamics. To use this function change 'mobile_robot_sim' to
%       'mobile_robot_sim_test' in the Simulation section in main_q1*.m.
%
% Inputs:
%       model:	A structure containing model parameters, dynamics model
%               handles, and the dimensions of the input and the state.
%       task:	A structure that describes the task to be
%               fulfilled/simulated. This includes parameters of the cost
%               function, initial and final time, sampling interval,
%               initial and goal states.
%       theta:	A (3 x (N-1)) array containing controller parameters at 
%               each time step, where N is the number of simulated
%               discrete-time steps.
%
% Output:
%       sim_out: A structure containing the simulation results including
%                the state and input trajectories under the given control
%                law, the cost at each stage and the accumulated cost over
%                the trajectory, and the controller parameters at each time
%                step.
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
% [20.01.31]    first version

function [ sim_out ] = mobile_robot_sim_test(model, task, theta, disturbance)
    % time
    dt = task.dt;
    times = task.start_time:task.dt:task.end_time;
    N = length(times);

	% state and input dimensions
    state_dim = 2;
    input_dim = 1;
    
    % initializations
    states = zeros(state_dim, N);
    inputs = zeros(input_dim, N-1);
    
    % cost Q and R
    Q_s = task.cost.params.Q_s;
    R_s = task.cost.params.R_s;
    Q_t = task.cost.params.Q_t;

    % initial state and cost
    cur_state = task.start_x;
    states(:,1) = cur_state;
    cost = ((cur_state-task.goal_x)'*Q_s*(cur_state-task.goal_x))*dt;
    cost_n = zeros(1,N);
    
    % simulate
    for k = 1:1:N-1
        % calculate input
        cur_input = theta(:,k)'*[1; cur_state];
        
        % derivatives of states
        state_dot = model.f_test(cur_state, cur_input, model, disturbance);
        
        % stage cost at time n
        cost_input = (cur_input'*R_s*cur_input)*dt;
        state_cost = ((cur_state-task.goal_x)'*Q_s*(cur_state-task.goal_x))*dt;
        cost_n(k) = cost_input + state_cost;

        % propagate forward
        cur_state = cur_state + task.dt*state_dot;
        
        % store results
        states(:,k+1) = cur_state;
        inputs(:,k) = cur_input;
        
        % compute cumulative cost
        if k == N-1
            % terminal state cost
            cost_state = (cur_state-task.goal_x)'*Q_t*(cur_state-task.goal_x);
        else
            % stage state cost
            cost_state = ((cur_state-task.goal_x)'*Q_s*(cur_state-task.goal_x))*dt;
        end
        cost = cost + cost_state + cost_input;
    end
    cost_n(end) = (cur_state-task.goal_x)'*Q_t*(cur_state-task.goal_x);

    % construct struct for plotting
    sim_out.t = times;
    sim_out.x = states;
    sim_out.u = inputs;
    sim_out.controller = theta;
    sim_out.cost = cost;
    sim_out.cost_n = cost_n;
end

