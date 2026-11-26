function [delta, flag] = lat_mpc(e, s_proj, v_x, delta_prev, ref, p, Ts)
%LAT_MPC  Yanal MPC (S3, LTV): Rajamani 2.5 hata dinamigi, giris delta, bozucu
%   psi_des' = v kappa. Ufuk hizi v_k: k = 0 olculen v_x, k >= 1 aracin
%   ufuktaki konumunda hiz profili (s_{k+1} = s_k + v_k Ts, v_{k+1} =
%   v_ref(s_{k+1})); A_k, E_k her ufuk adiminda v_k ile, B sabit. kappa
%   onizlemesi ayni s_k noktalarinda geometrik kappa (S2 hatalari ayni yola
%   gore). Turev bozuculari tam ayrik fark (profil ivmesi dahil); yalniz
%   k = 0 -> 1 adiminda olculen/profil hiz farki turevlenmez (sahte
%   sicrama). Regule edilen durum [e_y e_y' (e_psi - e_psi,ss) e_psi']:
%   e_psi,ss Rajamani 3.2 kalici yaw hatasi (geri beslemeyle sifirlanamaz;
%   hedef sicramasin diye yumusatilmis kappa_s ile). Bozucular: v kappa (psi_des'),
%   A_c(:,3) e_psi,ss, -psi_des'' (e_psi' = r - psi_des' oldugundan; egrilik
%   degisirken onizleme bunu gormeli) ve -e_psi,ss' (degisken donusumu).
%   e: Hata Hesabi cikisi [e_x e_y e_psi e_x' e_y' e_psi']. p: mpc_lat.
v  = zeros(1, p.Np + 1);  sk = v;  v(1) = max(v_x, p.vx_floor);  sk(1) = s_proj;
for k = 1:p.Np
    sk(k+1) = sk(k) + v(k)*Ts;
    v(k+1)  = max(periodic_interp(ref.v, sk(k+1), ref.ds), p.vx_floor);
end
a  = p.Cf + p.Cr;  b = p.Cf*p.lf - p.Cr*p.lr;  c = p.Cf*p.lf^2 + p.Cr*p.lr^2;
Bc = [0; p.Cf/p.m; 0; p.Cf*p.lf/p.Iz];
Ac = zeros(4, 4, p.Np);  Ec = zeros(4, 4, p.Np);
for k = 1:p.Np
    vk = v(k);
    Ac(:,:,k) = [0 1 0 0; 0 -a/(p.m*vk) a/p.m -b/(p.m*vk); 0 0 0 1; 0 -b/(p.Iz*vk) b/p.Iz -c/(p.Iz*vk)];
    Ec(:,:,k) = [[0; -b/(p.m*vk) - vk; 0; -c/(p.Iz*vk)], Ac(:,3,k), [0;0;0;-1], [0;0;-1;0]];
end
kap = periodic_interp(ref.kappa, sk, ref.ds).';   % S2 hatalariyla ayni (geometrik) yol
ks  = periodic_interp(ref.kappa_s, sk, ref.ds).';
K   = p.m*p.lf/(p.Cr*(p.lf + p.lr));               % e_psi,ss = -kappa_s (lr - K v^2)
ess = -ks.*(p.lr - K*v.^2);
w3  = diff(v.*kap)/Ts;  w3(1) = v(1)*(kap(2) - kap(1))/Ts;   % k = 0 -> 1: hiz farki turevlenmez
w4  = diff(ess)/Ts;     w4(1) = -(ks(2) - ks(1))/Ts*(p.lr - K*v(1)^2);
w   = [v(1:end-1).*kap(1:end-1); ess(1:end-1); w3; w4];
[delta, flag] = mpc_qp(Ac, Bc, Ec, w, [e(2); e(5); e(3) - ess(1); e(6)], Ts, p, delta_prev, []);
end
