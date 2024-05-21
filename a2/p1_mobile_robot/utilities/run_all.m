% run_all: Script for automating comparisons between the LQR and the
%          ILQR controller.
%
% Usage: Change simulation setup in task_design(). The commands below run
%        the LQR and ILQC controller scripts, and plot the responses for
%        compraison. By default, intermediate .mat data files are saved to
%        and read from './results/'.
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


main_p1_lqr; % run lqr
main_p2_ilqc; % run ilqc
plot_comparison; % plot results