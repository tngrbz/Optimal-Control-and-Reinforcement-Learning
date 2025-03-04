% get_lin_matrices: Function computing the Jacobian matrices for the
%                   mountain car dynamics model
%
% Inputs:
%       (state, input): operating point which the system dynamics is
%                       linearized about
%
% Outputs:
%       A:              Jacobian of dynamics with respect to state 
%                       evaluated at (state, input)
%       B:              Jacobian of dynamics with respect to input 
%                       evaluated at (state, input)
%
% --
% Control for Robotics
% Assignment 3
%
% --
% Technical University of Munich
% Learning Systems and Robotics Lab
%
% Course Instructor:
% Angela Schoellig
% angela.schoellig@tum.de
%
% Teaching Assistants: 
% SiQi Zhou: siqi.zhou@tum.de
% Lukas Brunke: lukas.brunke@tum.de
%
% --
% Revision history
% [20.03.07, SZ]    first version
% [22.03.02, SZ]    second 

function [A,B] = get_lin_matrices(state, input)
    % A
    a11 = 1+0.0075*sin(3*state(1));
    a12 = 1;
    a21 = 0.0075*sin(3*state(1));
    a22 = 1; 
    A = [a11, a12; a21, a22];
    
    % B
    B = [0.001; 0.001];
end

