function [thetak_ff, thetak_fb, sk_dash, sk, Sk] = update_policy(Ak, Bk, gk_dash, qk, Qk, rk, Rk, Pk, sknext_dash, sknext, Sk_next, xk_op, uk_op)
%UPDATE_POLICY Summary of this function goes here
%   Detailed explanation goes here
lk = rk + Bk' * sknext;
Gk = Pk + Bk' * Sk_next * Ak;
Hk = Rk + Bk' * Sk_next * Bk;
duk_ff = -Hk \ lk;
Kk = -Hk \ Gk;
sk_dash = gk_dash + sknext_dash + 1/2*duk_ff'*Hk*duk_ff + duk_ff'*lk;
sk = qk + Ak'*sknext + Kk'*Hk*duk_ff + Kk'*lk + Gk'*duk_ff;
Sk = Qk + Ak'*Sk_next*Ak + Kk'*Hk*Kk + Kk'*Gk + Gk'*Kk;
thetak_ff = uk_op + duk_ff - Kk*xk_op;
thetak_fb = Kk;

assert(size(lk, 1) == 1);
assert(size(Gk, 1) == 1 && size(Gk, 2) == 2);
assert(size(Hk, 1) == 1);
assert(size(Kk, 2) == 2);
assert(size(sk_dash, 1) == 1);
assert(size(sk, 1) == 2);
assert(size(Sk, 1) == 2);
end

