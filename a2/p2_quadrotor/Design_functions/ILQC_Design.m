% ILQC_Design: Implementation of the ILQC controller.
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
% This script is adapted from the course on Optimal & Learning Control for
% Autonomous Robots at the Swiss Federal Institute of Technology in Zurich
% (ETH Zurich). Course Instructor: Jonas Buchli. Course Webpage:
% http://www.adrlab.org/doku.php/adrl:education:lecture:fs2015
%
% --
% Revision history
% [20.01.31]    first version

function [Controller,cost] = ILQC_Design(Model,Task,Controller,Simulator)
% ILQC_DESIGN Implements the Iterative Linear Quadratic Controller (ILQC)
%    (see Ch. 4 notes for a formal description of the algorithm)

% Define functions that return the quadratic approximations of the cost 
% function at specific states and inputs (Eqns. (6)-(7) in handout)
% Example usage:
%     xn = [ x y z ... ]'; 
%     un = [ Fx Mx My Mz]';
%     t  = t;
%     Qm(xn,un) = Qm_fun(t,xn,un); 

% stage cost (l) quadratizations
g_ = Task.cost.l*Task.dt;
gbar_fun   = matlabFunction(  g_,'vars',{Task.cost.t,Task.cost.x,Task.cost.u});
% dl/dx
g_x = jacobian(Task.cost.l,Task.cost.x)'*Task.dt; % cont -> discr. time
gv_fun  = matlabFunction( g_x,'vars',{Task.cost.t,Task.cost.x,Task.cost.u});
% ddl/dxdx
g_xx = jacobian(g_x,Task.cost.x);
Qm_fun  = matlabFunction(g_xx,'vars',{Task.cost.t,Task.cost.x,Task.cost.u});
% dl/du
g_u = jacobian(Task.cost.l,Task.cost.u)'*Task.dt; % cont -> discr. time
rv_fun  = matlabFunction( g_u,'vars',{Task.cost.t,Task.cost.x,Task.cost.u});
% ddl/dudu
g_uu = jacobian(g_u,Task.cost.u);
Rm_fun  = matlabFunction(g_uu,'vars',{Task.cost.t,Task.cost.x,Task.cost.u});
% ddl/dudx
g_xu = jacobian(g_x,Task.cost.u)';
Pm_fun  = matlabFunction(g_xu,'vars',{Task.cost.t,Task.cost.x,Task.cost.u});

% terminal cost (h) quadratizations
gN_ = Task.cost.h;
gbarN_fun  = matlabFunction(  gN_,'vars',{Task.cost.x});
% dh/dx
gN_x = jacobian(Task.cost.h,Task.cost.x)';
gvN_fun = matlabFunction( gN_x,'vars',{Task.cost.x});
% ddh/dxdx
gN_xx = jacobian(gN_x,Task.cost.x);
QmN_fun = matlabFunction(gN_xx,'vars',{Task.cost.x});

% dimensions
n = length(Task.cost.x); % dimension of state space
m = length(Task.cost.u); % dimension of control input
N  = (Task.goal_time-Task.start_time)/Task.dt + 1; % number of time steps

% desired value function V* is of the form Eqn.(9) in handout
% V*(dx,n) = s + dx'*Sv + 1/2*dx'*Sm*dx
s    = zeros(1,N);
Sv   = zeros(n,N);
Sm   = zeros(n,n,N);

% Initializations
theta_temp = init_theta();
duff = zeros(m,1,N-1);
K    = repmat(theta_temp(2:end,:)', 1, 1, N-1);
sim_out.t = zeros(1, N);
sim_out.x = zeros(n, N);
sim_out.u = zeros(m, N-1);
X0 = zeros(n, N);
U0 = zeros(m, N-1);

% Shortcuts for function pointers to linearize systems dynamics:
% e.g. Model_Alin(x,u,Model_Param)
Model_Param = Model.param.syspar_vec;
Model_Alin  = Model.Alin{1}; 
Model_Blin  = Model.Blin{1}; 

%% [Problem 2.2 (d)] Implementation of ILQC controller
% Each ILQC iteration approximates the cost function as quadratic around the
% current states and inputs and solves the problem using DP.
i  = 1;
while ( i <= Task.max_iteration && ( norm(squeeze(duff)) > 0.01 || i == 1 ))
     
    %% Forward pass / "rollout" of the current policy
    % =====================================================================
    % [Todo] rollout states and inputs
    sim_out = Quad_Simulator(Model,Task,Controller);
    % =====================================================================
    
    % pause if cost diverges
    cost(i) = Calculate_Cost(sim_out, gbar_fun, gbarN_fun);
    fprintf('Cost of Iteration %2d (metric: ILQC cost function!): %6.4f \n', i-1, cost(i));
    
    if ( i > 1 && cost(i) > 2*cost(i-1) )
        fprintf('It looks like the solution may be unstable. \n')
        fprintf('Press ctrl+c to interrupt iLQG, or any other key to continue. \n')
        pause
    end
    
    %% Solve Riccati-like equations backwards in time
	% =====================================================================
    % [Todo] define nominal state and control input trajectories (dim by
    % time steps). Note: sim_out contains state x, input u, and time t
    %
    X0 = sim_out.x; % x_bar
    U0 = sim_out.u; % u_bar
    T0 = sim_out.t; % t
    % =====================================================================

    Q_t = Task.cost.Qmf;
    Q_s = Task.cost.Qm;
    R_s = Task.cost.Rm;
    state_goal = Task.goal_x;
    
    % =====================================================================
    % [Todo] Initialize the value function elements starting at final time
    % step (Problem 2.2 (g))
    %
    xf = X0(:,end); % final state when using current controller
    Sm(:,:,N) = QmN_fun(xf);
    Sv(:,N) = gvN_fun(xf);
    s(N) = gbarN_fun(xf);
    % =====================================================================
    
    % "Backward pass": Calculate the coefficients (s,Sv,Sm) for the value
    % functions at earlier times by proceeding backwards in time
    % (DP-approach)
    for k = (length(sim_out.t)-1):-1:1
        
        % state of system at time step n
        x0 = X0(:,k);
        u0 = U0(:,k);
        
        % =================================================================
        % [Todo] Discretize and linearize continuous system dynamics Alin
        % around specific pair (xO,uO). See exercise sheet Eqn. (18) for
        % details.
        %
        Alin = Model_Alin(x0,u0,Model_Param);
        Blin = Model_Blin(x0,u0,Model_Param);
        A = (eye(size(Alin,1)) + Alin*Task.dt);
        B = Blin*Task.dt;
        % =================================================================

        % =================================================================
        % [Todo] quadratize cost function
        % [Note] use function {gbar_fun, gv_fun, Qm_fun, rv_fun, Rm_fun,
        % Pm_fun} provided above.
        %
        t0 = T0(:,k);
        gbar = gbar_fun(t0,x0,u0);
        Qv = gv_fun(t0,x0,u0);
        Qm = Qm_fun(t0,x0,u0);
        Rv = rv_fun(t0,x0,u0);
        Rm = Rm_fun(t0,x0,u0);
        Pm = Pm_fun(t0,x0,u0);
        % =================================================================

        % =================================================================
        % [Todo] control dependent terms of cost function (Problem 2.2 (f))
        %
        l = Rv + B'*Sv(:,k+1);  % linear control dependent
        G = Pm + B'*Sm(:,:,k+1)*A;	% control and state dependent
        H = Rm + B'*Sm(:,:,k+1)*B;	% quadratic control dependent

        H = (H+H')/2; % ensuring H remains symmetric; do not delete!
        % =================================================================

        % =================================================================
        % [Todo] the optimal change of the input trajectory du = duff +
        % K*dx (Problem 2.2 (f))
        %
        duff(:,:,k) = -H\l;
        K(:,:,k) = -H\G;
        % =================================================================

        % =================================================================
        % [Todo] Solve Riccati-like equations for current time step n
        % (Problem 2.2 (g))
        %
        Sm(:,:,k) = Qm + A'*Sm(:,:,k+1)*A + K(:,:,k)'*H*K(:,:,k) + K(:,:,k)'*G + G'*K(:,:,k);
        Sv(:,k) = Qv + A'*Sv(:,k+1) + K(:,:,k)'*H*duff(:,:,k) + K(:,:,k)'*l + G'*duff(:,:,k);
        s(k) = gbar + s(k+1) + 1/2*duff(:,:,k)'*H*duff(:,:,k) + duff(:,:,k)'*l;
        % =================================================================

    end % of backward pass for solving Riccati equation
    
    % define theta_ff in this function
    Controller.theta = Update_Controller(X0, U0, duff, K);
    
    i = i+1;
end

% simulating for the last update just to calculate the final cost
sim_out    = Simulator(Model,Task,Controller);
cost(i) = Calculate_Cost(sim_out, gbar_fun, gbarN_fun);
fprintf('Cost of Iteration %2d: %6.4f \n', i-1, cost(i));
end



function theta = Update_Controller(X0,U0,dUff,K)
% UPDATE_CONTROLLER Updates the controller after every ILQC iteration
%
%  X0  - state trajectory generated with controller from previous 
%        ILQC iteration.
%  UO  - control input generated from previous ILQC iteration.
%  dUff- optimal update of feedforward input found in current iteration
%  K   - optimal state feedback gain found in current iteration
%
%  The updated control policy has the following form:
%  U1 = U0 + dUff + K(X - X0)
%     = U0 + dUff - K*X0 + K*X
%     =      Uff         + K*x
%  
%  This must be brought into the form 
%  U1 = theta' * [1,x']   

%% Update Controller
% input and state dimensions
n  = size(X0,1); % dimension of state
m = size(U0,1); % dimension of control input
N  = size(X0,2); % number of time steps

% initialization
theta = init_theta();
theta_fb = zeros(n, m, N-1);
theta_ff = repmat(theta(1,:), 1, 1, N-1);

% =========================================================================
% [Todo] feedforward control input
%
U0 = permute(U0, [3, 1, 2]);
dUff = permute(dUff, [2, 1, 3]);
KX0 = zeros(1, m, N-1);
for i = 1:1:N-1
    KX0(:,:,i) = K(:,:,i)*X0(:,i);
end
%
theta_ff = U0 + dUff - KX0;
% =========================================================================

% feedback gain of control input (size: n * m * N-1)
theta_fb = permute(K,[2 1 3]);      

% puts below (adds matrices along first(=row) dimension). 
% (size: (n+1) * m * N-1)
theta = [theta_ff; theta_fb];  

end


function cost = Calculate_Cost(sim_out, gbar_fun, gbarN_fun)
% CALCULATE_COST: calcules the cost of current state and input trajectory
% for the current ILQC cost function. Not neccessarily the same as the LQR
% cost function.

X0 = sim_out.x(:,1:end-1);
xf = sim_out.x(:,end);
U0 = sim_out.u;
T0 = sim_out.t(1:end-1);

cost = sum(gbar_fun(T0,X0,U0)) + gbarN_fun(xf);
end
