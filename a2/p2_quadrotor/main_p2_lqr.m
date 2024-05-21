% main_p2_lqr: Main script for Problem 2.2 LQR controller design.
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

close all;
clear all;
clc;

%% General
% add subdirectories
addpath(genpath(pwd));

% define task
Task = Task_Design();

% load the dynamic model of the quadcopter
load('Quadrotor_Model.mat', 'Model'); % save as structure "Model" 

% define cost function
Task.cost = Cost_Design( Model.param.mQ, Task );

% save directory
save_dir = './Results/';

% flags
plot_on = true;
save_on = true;

%% Initial LQR Controller Design
% [Problem 2.2 (a)-(c)] Fill in the missing parts in ...
% LQR_Design(Model, Task)
[Initial_Controller, Cost_LQR] = LQR_Design(Model, Task);

%% Simulation
Sim_Out_LQR = Quad_Simulator(Model, Task, Initial_Controller);
disp('LQR controller performance:');
fprintf('Cost with LQR controller (metric: LQR cost function!): J* = %.3f \n', Cost_LQR);
fprintf('Start Quadcopter position: x = %.3f, y = %.3f, z = %.3f \n', Sim_Out_LQR.x(1:3,1));
fprintf('Final Quadcopter position: x = %.3f, y = %.3f, z = %.3f \n\n', Sim_Out_LQR.x(1:3,end));

%% Visualization of LQR controller
if plot_on
    Visualize2(Sim_Out_LQR, Model.param);
end

%% Save Results and Figures
if save_on
    if ~exist(save_dir, 'dir')
       mkdir(save_dir); 
    end
    
    % save controller and simulation results
	save(strcat(save_dir, 'lqr_controller'), 'Initial_Controller', 'Sim_Out_LQR', 'Task');
    
	% save visualization plot for report
    if plot_on
        saveas(gcf, strcat(save_dir, 'lqr_controller'), 'png');
    end
end