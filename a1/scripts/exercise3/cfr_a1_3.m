% cfr_a1_3: Main script for Problem 1.3 Approximate Dynamic Programming.
%
% adapted from: Borrelli, Francesco: "ME 231 Experiential Advanced Control
% Design I"
%
% --
% Control for Robotics
% Summer 2023
% Assignment 1
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
% [22.01.17, LB]    first version
% [22.01.23, LB]    added 2 (c) to the code, removed N
%
% --
% Revision history
% [22.01.17, LB]    first version
% [22.01.24, LB]    updated horizon and initial state

clear all
close all
clc

%% set up system

% inverted pendulum parameters
l = 1.0; % length
g = 9.81; % gravitational constant
m = 1.0; % mass

% create inverted pendulum system
sys = InvertedPendulum(l, g, m);

% controller parameters
Q = diag([1, 0.1]);
R = 1;
N = 25;

% linearization point
x_up = [pi; 0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO: YOUR CODE HERE - Exercise 3 (a)
A = [0 1; -g/l 0];
b = [0; 1/(m*l^2)];
c = zeros(1, size(A, 1));
d = 0;
Ts = 0.1;
sys1 = ss(A, b, c, d);
sysd = c2d(sys1, Ts);
Ad = sysd.A;
Bd = sysd.B;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% cost functions

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO: YOUR CODE HERE - Exercise 3 (b)
%{
function Jx = costfcn(u, x_0)
    x = x_0;
    Jx = 0;
    for i=1:N-1
       Jx = Jx + x'*Q*x + r*u^2;
        x = Ad*x + Bd*u;
    end
    Jx = Jx + x'*Q*x;
end
%}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% calculate optimal control using dynamic programming and gridding

% grid state-space
num_points_x1 = 10;
num_points_x2 = 5;
X1 = linspace(-pi/4, pi/4, num_points_x1);
X2 = linspace(-pi/2, pi/2, num_points_x2);

% allocate arrays for optimal control inputs and cost-to-go 
U = zeros(num_points_x1, num_points_x2);
J = zeros(num_points_x1, num_points_x2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO: YOUR CODE HERE - Exercise 3 (c)
x0 = x_up + [-pi/6; 0];
[ux, Jval] = fminunc(@(u)costfcn(u, x0, N, Q, R, Ad, Bd), zeros(N, 1));
% with u and sys find next states and cost in each state
% interpolate u and J accordign to states
Jx = zeros(N+1, 1);
x_rec = zeros(2,N);
x_rec(:,1) = x0;
for i=1:N-1
    Jx(i+1) = Jx(i) + x_rec(:,i)'*Q*x_rec(:,i) +R*ux(i)^2;
    x_rec(:,i+1) = Ad*x_rec(:,i) + Bd*ux(i);
end
    Jx(N+1) = Jx(N) + x_rec(:,N)'*Q*x_rec(:,N);
assert(Jval == Jx(N+1));
%% Interpolation

% Initialize cost-to-go and optimal input matrices
J = zeros(length(X1), length(X2), N+1); % Cost-to-go matrix
U = zeros(length(X1), length(X2), N); % Optimal input matrix

% Set the final cost-to-go values
J(:,:,N+1) = ...; % Set the final cost-to-go values (e.g., J_N(x1,x2))

% Perform dynamic programming backward recursion
for k = N:-1:1
    for i = 1:length(X1)
        for j = 1:length(X2)
            % Current state
            x1 = X1(i);
            x2 = X2(j);
            
            % Interpolate cost-to-go from next time step
            J_interp = @(u) interp2(X1, X2, J(:,:,k+1), x1 + dt * A * [x1; x2] + dt * B * u(1), x2);
            
            % Solve unconstrained optimization problem
            u_opt = fminunc(J_interp, 0); % Minimize the interpolated cost-to-go
            
            % Calculate cost-to-go and optimal input
            J(i, j, k) = J_interp(u_opt);
            U(i, j, k) = u_opt;
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% plot optimal control and cost-to-go
figure
subplot(1, 2, 1)
surf(X1, X2, U')
xlabel('x_1')
ylabel('x_2')
zlabel('u')
subplot(1, 2, 2)
surf(X1, X2, J')
xlabel('x_1')
ylabel('x_2')
zlabel('J')

%% apply control law and simulate inverted pendulum
% create the controlled inverted pendulum system
control_sys = InvertedPendulum(l, g, m, X1, X2, U, x_up);

% initial condition
x0 = x_up + [-pi/6; 0];

% duration of simulation
t = [0, 10];

% simulate control system
[t, x] = ode45(@control_sys.controlled_dynamics, t, x0);

% determine control inputs from trajectory
u = zeros(size(t));
for i = 1 : length(t)
    u(i) = control_sys.mu(x(i, :)' - x_up);
end

%% plot state and input trajectories
figure
subplot(2, 1, 1)
hold on
plot(t, x(:, 1))
plot(t, x(:, 2))
xlabel('t')
ylabel('x_1 and x_2')
hold off
legend('\theta','d\theta/dt')
grid on
subplot(2, 1, 2)
plot(t, u)
xlabel('t')
ylabel('u')
grid on

%% Function must be defined here!!
function Jx = costfcn(u, x_0, N, Q, R, Ad, Bd)
    x = x_0;
    Jx = 0;
    for i=1:N-1
       Jx = Jx + x'*Q*x + R*u(i)^2;
        x = Ad*x + Bd*u(i);
    end
    Jx = Jx + x'*Q*x;
end