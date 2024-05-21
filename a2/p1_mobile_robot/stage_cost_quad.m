function [gk_dash,qk,Qk,rk,Rk,Pk] = stage_cost_quad(Qs,Rs,x_goal,dt,xk_op,uk_op)
%STAGE_COST_QUAD Summary of this function goes here
%   Detailed explanation goes here
gk_dash = 1/2/dt*(xk_op - x_goal)'*Qs*(xk_op - x_goal) + uk_op*Rs*uk_op;
qk = 1/dt*Qs*(xk_op - x_goal);
Qk = 1/dt*Qs;
rk = 1/dt*Rs*uk_op;
Rk = 1/dt*Rs;
Pk = 0;

assert(size(gk_dash, 1) == 1);
assert(size(qk, 1) == 2);
assert(size(Qk, 2) == 2 && size(Qk, 1) == 2);
assert(size(rk, 1) == 1);
assert(size(Rk, 1) == 1);
end

