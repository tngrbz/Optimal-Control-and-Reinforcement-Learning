% main_p1_ilqc: Main script for Problem 2.1 ILQC controller design.
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

clear all;
close all;

%% General
% add subdirectories
addpath(genpath(pwd));

% add task
task_ilqc = task_design();
N = length(task_ilqc.start_time:task_ilqc.dt:task_ilqc.end_time);

% add model
const_vel = 1; % assume constant forward speed
model = generate_model(const_vel);

% save directory
save_dir = './results/';

% initialize controller
load(strcat(save_dir, 'lqr_controller'));
controller_ilqc = controller_lqr;

% flags
plot_on = true;
save_on = true;

%% [Problem 2.1 (j)] Iterative Linear Quadratic Controller
% =========================== [TODO] ILQC Design ==========================
% Design an ILQC controller based on the linearized dynamics and
% quadratized costs. The cost function of the problem is specified in
% 'task_ilqc.cost' via the method 'task_design()'.
%
%
% =========================================================================
dt = task_ilqc.dt;         % discrete time step

% start and final states
start_x = task_ilqc.start_x;   % [y; h]
goal_x = task_ilqc.goal_x;   % [y; h]

% stage cost function parameters
Qs = task_ilqc.cost.params.Q_s;
Rs = task_ilqc.cost.params.R_s;

% terminal cost function parameters
Qt = task_ilqc.cost.params.Q_t;

%theta = zeros(3,N-1);
ilqr_iter = 0;
s_dash = zeros(1, N);
s = zeros(2, N);
S = zeros(4, N);
 while ilqr_iter < task_ilqc.max_iteration
    ilqr_iter = ilqr_iter + 1;
    sim_out = mobile_robot_sim(model, task_ilqc, controller_ilqc);
    x_op = sim_out.x;
    assert(size(x_op, 1) == 2);
    u_op = sim_out.u;
    assert(size(u_op, 1) == 1);
    [gN_dash, qN, QN] = terminal_cost_quad(Qt, goal_x, x_op(:,N));
    s_dash(1,N) = gN_dash;
    s(:,N) = qN;
    S(:,N) = QN(:);
    for i=N-1:-1:1
        [Ak, Bk] = mobile_robot_lin(x_op(:,i), u_op(i), dt, const_vel);
        [gk_dash, qk, Qk, rk, Rk, Pk] = stage_cost_quad(Qs, Rs, goal_x, dt, x_op(:,i), u_op(i));
        Sk_next = reshape(S(:,i+1), [2 2]);
        [thetak_ff, thetak_fb, sk_dash, sk, Sk] = update_policy(Ak, Bk, gk_dash, qk, Qk, rk, Rk, Pk,s_dash(1,i+1), s(:,i+1), Sk_next, x_op(:,i), u_op(i));
        controller_ilqc(:, i) = [thetak_ff; thetak_fb'];
        s_dash(1, i) = sk_dash;
        s(:,i) = sk;
        S(:,i) = Sk(:);
    end
 end
%controller_ilqc = theta;
%% Simulation
sim_out_ilqc = mobile_robot_sim(model, task_ilqc, controller_ilqc);
fprintf('\n\ntarget state [%.3f; %.3f]\n', task_ilqc.goal_x);
fprintf('reached state [%.3f; %.3f]\n', sim_out_ilqc.x(:,end));

%% Plots
if plot_on
    plot_results(sim_out_ilqc);
end

%% Save controller and simulation results
if save_on
    if ~exist(save_dir, 'dir')
       mkdir(save_dir); 
    end
    
    % save controller and simulation results
	save(strcat(save_dir, 'ilqc_controller'), 'controller_ilqc', ...
        'sim_out_ilqc', 'task_ilqc'); 
end