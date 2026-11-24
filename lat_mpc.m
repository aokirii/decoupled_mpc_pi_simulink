function [delta, flag] = lat_mpc(e, s_proj, v_x, delta_prev, ref, p, Ts)
%LAT_MPC  Yanal MPC (S3): Rajamani 2.5 hata dinamigi, giris delta, bozucu
%   psi_des' = v_x kappa; kappa onizlemesi s_proj + v_x k Ts noktalarinda
%   geometrik kappa (S2 hatalari ayni yola gore). Regule edilen
%   durum [e_y e_y' (e_psi - e_psi,ss) e_psi']: e_psi,ss Rajamani 3.2 kalici
%   yaw hatasi (geri beslemeyle sifirlanamaz; hedef sicramasin diye
%   yumusatilmis kappa_s ile). Bozucular: v kappa (psi_des'),
%   A_c(:,3) e_psi,ss, -psi_des'' (e_psi' = r - psi_des' oldugundan; egrilik
%   degisirken onizleme bunu gormeli) ve -e_psi,ss' (degisken donusumu).
%   e: Hata Hesabi cikisi [e_x e_y e_psi e_x' e_y' e_psi']. p: mpc_lat.
v  = max(v_x, p.vx_floor);
a  = p.Cf + p.Cr;  b = p.Cf*p.lf - p.Cr*p.lr;  c = p.Cf*p.lf^2 + p.Cr*p.lr^2;
Ac = [0 1 0 0; 0 -a/(p.m*v) a/p.m -b/(p.m*v); 0 0 0 1; 0 -b/(p.Iz*v) b/p.Iz -c/(p.Iz*v)];
Bc = [0; p.Cf/p.m; 0; p.Cf*p.lf/p.Iz];
Ec = [0; -b/(p.m*v) - v; 0; -c/(p.Iz*v)];
sk  = s_proj + v*Ts*(0:p.Np);
kap = periodic_interp(ref.kappa, sk, ref.ds).';   % S2 hatalariyla ayni (geometrik) yol
ess = -periodic_interp(ref.kappa_s, sk, ref.ds).'*(p.lr - p.m*v^2*p.lf/(p.Cr*(p.lf + p.lr)));
w   = [v*kap(1:end-1); ess(1:end-1); v*diff(kap)/Ts; diff(ess)/Ts];
[delta, flag] = mpc_qp(Ac, Bc, [Ec, Ac(:,3), [0;0;0;-1], [0;0;-1;0]], w, ...
                       [e(2); e(5); e(3) - ess(1); e(6)], Ts, p, delta_prev, []);
end
