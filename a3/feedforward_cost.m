function [x] = feedforward_cost(x)
%   Calculate the feedforward of the given linearized system equations and 
%   with corresponding feature states it returns the cost.

    load(params2.mat);
    % Extract input and state from the vector x
    u = x(1:n);
    STATES_CROSSTERMS = x(n+1:end);
    odd_idx = 1:2:2*n;
    even_idx = odd_idx + 1;
    x1 = STATES_CROSSTERMS(odd_idx);
    x2 = STATES_CROSSTERMS(even_idx);

    for i=1:n
        next_idx = i+1;
        x_next = A*[x1(i);x2(i)] + B*x(i);
        x1(next_idx) = x_next(1);
        x2(next_idx) = x_next(2);
        x(n+2*i-1:n+2*i) = x_next;
    end
    
end

