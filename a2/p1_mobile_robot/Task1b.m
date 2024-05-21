Q = diag([1, 1]); % Qt = Qs = Q
Rs = 20;
A = [0 1; 0 0];
B = [0; 1];
sys = ss(A,B,[],[]);
[K, S, CLP] = lqr(sys, Q, Rs);