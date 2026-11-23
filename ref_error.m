function [e_x, e_y, e_psi, e_x_dot, e_y_dot, e_psi_dot, s_veh, s_proj, idx, lap, ...
          v_ref_s, v_ref_p, kappa_p, theta_p, s_dot] = ref_error( ...
          X, Y, psi, v_x, v_y, r, s_ref, idx_prev, lap_prev, s_proj_prev, ref, win_fwd, win_back)
%REF_ERROR  Yola izdusum ve takip hatalari (S2, Rajamani 2.5 isaret kurali).
%   Girisler: arac durumu X, Y, psi, v_x, v_y, r; acilmis s_ref; geri
%   besleme durumlari idx_prev (< 1 = ilk adim, tum yolda arama), lap_prev,
%   s_proj_prev; ref = reference_path ciktisi; pencere win_fwd / win_back
%   (dugum sayisi, varsayilan 120 / 20).
%   1) En yakin dugum: ilk adimda global, sonra idx_prev - win_back ...
%      idx_prev + win_fwd penceresi (mod N).
%   2) Komsu iki segmente dik izdusum, yakin olan secilir; sonra izdusum
%      noktasi, ara degerli teget theta_p'ye dik olacak sekilde 3 Newton
%      adimiyla duzeltilir (e_x, s icin; buyuk e_y'de kiris yonu hatasini
%      kaldirir). s_proj surekli, [0, L) (kayan noktada mod sonucu L'ye
%      esit olabilir; path_at bunu son segmentin t = 1 ucu olarak okur,
%      tur sayaci etkilenmez).
%   3) e_y = -(X-Xp) sin(theta_p) + (Y-Yp) cos(theta_p)   (sola pozitif)
%          + kappa_p t (1-t) ds^2 / 2   (kiris -> yerel yay duzeltmesi)
%      e_psi = wrap(psi - theta_p)
%      e_y_dot = v_x sin(e_psi) + v_y cos(e_psi)
%      s_dot = (v_x cos(e_psi) - v_y sin(e_psi)) / (1 - kappa_p e_y)
%      e_psi_dot = r - kappa_p s_dot
%   4) Tur: s_proj - s_proj_prev < -L/2 -> lap+1, > L/2 -> lap-1. Ilk adimda
%      lap = round((s_ref - s_proj)/L) (s_veh, s_ref'e en yakin tur).
%      s_veh = s_proj + lap L;  e_x = s_ref - s_veh (+ = arac geride);
%      e_x_dot = v_ref(s_ref) - s_dot.
%   Yoldan uzak arac icin de sonlu deger doner: Newton adiminda ve s_dot
%   paydasinda 1 - kappa_p e_y >= 0.05 tabani (taban yalniz egrilik
%   merkezine 0.05 R'den yakin, fiziksel olmayan bolgede devreye girer).
%#codegen
N  = numel(ref.x);
ds = ref.ds;
L  = ref.L;

%% 1) En yakin dugum
ip   = round(idx_prev);
best = inf;
idx  = 1;
if ip < 1
    for j = 1:N
        d2 = (ref.x(j) - X)^2 + (ref.y(j) - Y)^2;
        if d2 < best, best = d2; idx = j; end
    end
else
    for k = -win_back:win_fwd
        j  = mod(ip - 1 + k, N) + 1;
        d2 = (ref.x(j) - X)^2 + (ref.y(j) - Y)^2;
        if d2 < best, best = d2; idx = j; end
    end
end

%% 2) Komsu segmentlere izdusum + Newton duzeltmesi
ia = mod(idx - 2, N) + 1;
ib = mod(idx, N) + 1;
[t1, d1] = seg_proj(ref.x(ia), ref.y(ia), ref.x(idx), ref.y(idx), X, Y);
[t2, d2] = seg_proj(ref.x(idx), ref.y(idx), ref.x(ib), ref.y(ib), X, Y);
if d1 < d2
    s_p = (ia - 1 + t1) * ds;
else
    s_p = (idx - 1 + t2) * ds;
end
s_p = mod(s_p, L);
for it = 1:3
    [xp, yp, thp, kp] = path_at(ref, s_p);
    dx  = X - xp;
    dy  = Y - yp;
    eyt = -dx*sin(thp) + dy*cos(thp);
    stp = (dx*cos(thp) + dy*sin(thp)) / max(1 - kp*eyt, 0.05);
    stp = min(max(stp, -ds), ds);
    s_p = mod(s_p + stp, L);
end
s_proj = s_p;
[xp, yp, theta_p, kappa_p, tp] = path_at(ref, s_proj);

%% 3) Hatalar
% kiris yerine yerel yay: egri, kirisin kappa_p t(1-t) ds^2/2 sagindadir
e_y   = -(X - xp)*sin(theta_p) + (Y - yp)*cos(theta_p) + kappa_p*tp*(1 - tp)*ds^2/2;
e_psi = wrap_pi(psi - theta_p);
e_y_dot   = v_x*sin(e_psi) + v_y*cos(e_psi);
s_dot     = (v_x*cos(e_psi) - v_y*sin(e_psi)) / max(1 - kappa_p*e_y, 0.05);
e_psi_dot = r - kappa_p*s_dot;

%% 4) Tur sayaci ve boyuna hata
if ip < 1
    lap = round((s_ref - s_proj) / L);
else
    lap = lap_prev;
    if s_proj - s_proj_prev < -L/2
        lap = lap + 1;
    elseif s_proj - s_proj_prev > L/2
        lap = lap - 1;
    end
end
s_veh   = s_proj + lap*L;
e_x     = s_ref - s_veh;
v_ref_s = periodic_interp(ref.v, s_ref, ds);
v_ref_p = periodic_interp(ref.v, s_proj, ds);
e_x_dot = v_ref_s - s_dot;
end

%% ======================================================================
function [t, d2] = seg_proj(xa, ya, xb, yb, X, Y)
% a -> b segmentine dik izdusum parametresi t in [0, 1] ve uzaklik karesi
ux = xb - xa;  uy = yb - ya;
t  = ((X - xa)*ux + (Y - ya)*uy) / max(ux*ux + uy*uy, eps);
t  = min(max(t, 0), 1);
d2 = (X - xa - t*ux)^2 + (Y - ya - t*uy)^2;
end

function [xp, yp, thp, kp, t] = path_at(ref, s)
% s in [0, L) noktasinda konum, yon (wrap'li) ve egrilik, lineer ara deger;
% t = segment ici parametre
N  = numel(ref.x);
u  = s / ref.ds;
j  = min(floor(u), N - 1);
t  = u - j;
a  = j + 1;
b  = mod(j + 1, N) + 1;
xp  = ref.x(a) + t*(ref.x(b) - ref.x(a));
yp  = ref.y(a) + t*(ref.y(b) - ref.y(a));
thp = wrap_pi(ref.theta(a) + t*wrap_pi(ref.theta(b) - ref.theta(a)));
kp  = ref.kappa(a) + t*(ref.kappa(b) - ref.kappa(a));
end

function a = wrap_pi(a)
a = mod(a + pi, 2*pi) - pi;
end
