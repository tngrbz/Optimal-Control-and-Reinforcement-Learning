% state_index_lookup: Function for finding index of discrete state
%
% Inputs:
%       STATES:  An array defining the discrete states
%       vector:  An array containing a set of state vectors for which the
%                indeces are to be found
%
% Outputs:
%       index:   An array of state indeces corresponding to that given by
%                the input argument 'vector'
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
% --
% Revision history
% [20.03.07, SZ]    first version

function [index] = state_index_lookup(STATES, vector)
    % Dimensions
    state_dim = size(STATES, 1);
    num_vectors = size(vector, 2);
    
    % Find indeces
    index = [];
    for v = 1:1:num_vectors
        vector_temp = vector(:,v);
        temp = true;
        for l = 1:1:state_dim
            temp = temp & STATES(l,:) == vector_temp(l);
        end
        index = [index, find(temp)];
    end
end
