% monte_carlo: Function solving the given MDP using the on-policy Monte
%              Carlo method
%
% Inputs:
%       world:                  A structure defining the MDP to be solved
%       epsilon:                A parameter defining the 'sofeness' of the 
%                               epsilon-soft policy
%       k_epsilon:              The decay factor of epsilon per iteration
%       omega:                  Learning rate for updating Q
%       training_iterations:    Maximum number of training episodes
%       episode_length:         Maximum number of steps in each training
%                               episodes
%
% Outputs:
%       Q:                      An array containing the action value for
%                               each state-action pair 
%       policy_index:           An array summarizing the index of the
%                               optimal action index at each state
%
% --
% Control for Robotics
% Assignment 4
%
% --
% Technical University of Munich
% Learning Systems and Robotics Lab
%
% Course Instructor:
% Angela Schoellig
% angela.schoellig@tum.de
%
% Teaching Assistants: a
% SiQi Zhou: siqi.zhou@tum.de
% Lukas Brunke: lukas.brunke@tum.de
%
% This script is adapted from the course on Optimal & Learning Control for
% Autonomous Robots at the Swiss Federal Institute of Technology in Zurich
% (ETH Zurich). Course Instructor: Jonas Buchli. Course Webpage:
% http://www.adrlab.org/doku.php/adrl:education:lecture:fs2015
%
% --
% Revision history
% [20.03.07, SZ]    first version

function [Q, policy_index] = ...
    monte_carlo(world, epsilon, k_epsilon, omega, training_iterations, episode_length)
    %% Initialization
    % MDP
    mdp = world.mdp;
    gamma = mdp.gamma;

    % States
    STATES = mdp.STATES;
    ACTIONS = mdp.ACTIONS;

    % Dimensionts
    num_states = size(STATES, 2);
    num_actions = size(ACTIONS, 2);

    % Create object for incremental plotting of reward after each episode
    windowSize = 10; %Sets the width of the sliding window fitler used in plotting
    plotter = RewardPlotter(windowSize);

    % Initialize Q
    Q = zeros(num_states, num_actions);
    
    % [TODO] Initialize epsilon-soft policy
    % policy = ...; % size: num_states x num_actions

    %% On-policy Monte Carlo Algorithm (Section 2.9.3 of [1])
    for train_loop = 1:1:training_iterations
        %% [TODO] Generate a training episode

        % while ... % episode termination criteria
            % episode_index = episode_index + 1;

            % Sample current epsilon-soft policy
            % action = ...;

            % Interaction with environment
            % [next_state_index, ~, reward] = one_step_gw_model(world, cur_state_index, action, 1);

            % Log data for the episode
            % ...
        % end

        % Update Q(s,a)
        % Q = ...;

        %% [TODO] Update policy(s,a)
        % policy = ...;

        %% [TODO] Update the reward plot
        % EpisodeTotalReturn = ...; % Sum of the reward obtained during the episode
        plotter = UpdatePlot(plotter, EpisodeTotalReturn);
        drawnow;
        pause(0.1);

        %% Decrease the exploration
        % Set k_epsilon = 1 to maintain constant exploration
        epsilon = epsilon * k_epsilon;
    end
    
    % Return deterministic policy for plotting
    [~, policy_index] = max(policy, [], 2);
end
