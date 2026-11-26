function [F_x, flag] = lon_mpc(e_x, s_ref, s_proj, v_x, a_y, F_prev, ref, p, Ts)
%LON_MPC  Boyuna MPC (S3): x = [e_x, e_v], e_v = v_x - v_ref(s_ref), giris
%   a = F_x/m (QP ivme biriminde cozulur; kuvvet biriminde kotu olcekli).
%   e_x' = -e_v (v_x ~ s_dot), e_v' = a - F_res(v_x)/m - a_drag - v_ref'.
%   v_ref onizlemesi s_ref'ten, s_k+1 = s_k + v_ref(s_k) Ts; F_res mevcut
%   v_x'te ileri besleme. Viraj suruklemesi (kararli viraj, dogrusal lastik):
%   a_drag = (F_yf a_f + F_yr a_r)/m = m (v^2 kappa)^2 (lr^2/Cf + lf^2/Cr)/L^2
%   (-F_yf sin(delta)/m ile +v_y r toplami), kappa onizlemesiyle.
%   Kayma onlemi: (1) yumusak sinir v_x,k <= v_ref(s_proj + v_x k Ts)
%   (aracin konumundaki tutunma sinirli yol hizi); (2) surtunme elipsi,
%   olculen yanal ivmeyle (a_y = v_x r), iki sinira:
%   a_min,max sqrt(1 - (a_y/ay_lim)^2) (patinaj ve blokaj).
%   ay_lim = 0.75 mu g. p: mpc_lon.
sr = zeros(1, p.Np + 1);  sr(1) = s_ref;
for k = 1:p.Np, sr(k+1) = sr(k) + periodic_interp(ref.v, sr(k), ref.ds)*Ts; end
vr = periodic_interp(ref.v, sr, ref.ds).';
Fres = 0.5*p.rho_air*p.CdA*v_x*abs(v_x) + p.C_rr*p.m*p.g;
vp = periodic_interp(ref.v, s_proj + v_x*Ts*(1:p.Np), ref.ds).';
kp = periodic_interp(ref.kappa, s_proj + v_x*Ts*(0:p.Np-1), ref.ds).';
p.ulim = p.ulim*sqrt(max(0, 1 - (a_y/p.ay_lim)^2));
p.ulim = [min(p.ulim(1), F_prev/p.m + p.dumax), max(p.ulim(2), F_prev/p.m - p.dumax)];   % Delta-a bandiyla kesissin (uygulanabilirlik)
[a, flag] = mpc_qp([0 -1; 0 0], [0; 1], [0; 1], -Fres/p.m - p.m*v_x^4*kp.^2*(p.lr^2/p.Cf + p.lf^2/p.Cr)/(p.lf + p.lr)^2 - diff(vr)/Ts, [e_x; v_x - vr(1)], ...
                   Ts, p, F_prev/p.m, vp - vr(2:end));
F_x = p.m*a;
end
