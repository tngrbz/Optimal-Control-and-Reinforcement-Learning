% get_cost: Function constructing Hessian of cost function
%
% Inputs:
%       r:              Non-negative scalar penalizing input
%       Q:              Symmetric positive semidefinite matrix penalizing
%                       errors in state
%       n_lookahead:    Length of MPC prediction horizon
%
% Outputs:
%       S:              Hessian matrix of cost function
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
% [22.03.02, SZ]    second version

function [S] = get_cost(r, Q, n_lookahead)
    % cost function
    % cost matrix S is eventually nxn for the 
    % first part and 2nx2n for the second part
    % so 3nx3n
    S = [];
    for i = 1:1:n_lookahead
        S = blkdiag(S, r);
    end

    for i = 1:1:n_lookahead
        S = blkdiag(S, Q);
    end
end