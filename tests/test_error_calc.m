function test_error_calc()
%TEST_ERROR_CALC  S2-A hata hesabi, referans noktasi ve bitis testi (T4, T5).
%   T4 ref_error: analitik yol (cember, R_c = 8 yuvarlatilmis kare) uzerinde
%      bilinen (s, e_y, e_psi, e_x) ile kurulan sentetik pozlar (dikis,
%      yaylar, duzlukler); global ilk arama ve pencere; yoldan uzak arac;
%      dikisten ileri ve geri gecen duzgun yorungelerde tur sayaci, pencere
%      surekliligi, e_y_dot / e_psi_dot / s_dot tutarliligi.
%   T5 ref_advance: bagimsiz ozyinelemeyle esitlik, ode45 surekli cozumune
%      sapma; ref_done: tur ve durma olcutleri, stop_enable = 0.
%   Olcut asilirsa error. Kullanim (proje kokunden):
%       addpath tests; test_error_calc

root = fileparts(fileparts(mfilename('fullpath')));
oldpath = addpath(root);
cleanup = onCleanup(@() path(oldpath)); %#ok<NASGU>

prm = struct('ds_target', 0.1, 'corner_radius', 0, 'interp_method', 'spline', 'v_max', 8, ...
    'ay_max', 0.85*0.75*9.81, 'ax_max', 3, 'ax_min', -6, 'v_min', 2, 'kappa_win', 3);
wf = 120;  wb = 20;  Ts = 0.02;
res = {};

[wp, sp] = load_map(fullfile(root, 'maps', 'YuvarlakMap.m'));
c = reference_path(wp, sp, prm);
[wp, sp] = load_map(fullfile(root, 'maps', 'KareMap.m'));
p8 = prm;  p8.corner_radius = 8;
k = reference_path(wp, sp, p8);
so = k.y(1) - 68;                         % k'nin s = 0 noktasi, sag kenarda teget noktasindan
assert(abs(k.x(1) - 140) < 1e-9, 'Kare s = 0 sag kenarda bekleniyordu');
paths = {struct('n', 'Circle', 'r', c, 'tr', @(s) truth_circle(s)), ...
         struct('n', 'Kare',   'r', k, 'tr', @(s) truth_rsq(s, so))};

%% ---------------- T4a: sentetik pozlar ----------------
Lk = k.L;  Lq = 64 + 4*pi;                 % kare kenar + yay
sl = {[0, 0.03, c.L - 0.04, pi*120/2, 400.123], ...
      [0, 0.04, Lk - 0.03, ...                                    % dikis
       (0:3)*Lq + 64 + 2*pi - so, (0:3)*Lq + 64 + 1.5 - so, ...    % yay ortasi, yaya 1.5 m
       (0:3)*Lq + 20 - so, 1*Lq + 40 - so]};                       % duzlukler
eyl = [-1.5 0 0.8];  epl = [-0.2 0 0.1];  dsl = [-2.5 0 1.7];
for q = 1:2
    P = paths{q};  r = P.r;  emax = zeros(1,3);  ewin = 0;  nlap = 0;  ntot = 0;
    for s0 = mod(sl{q}, r.L)
        [p, th] = P.tr(s0);
        nrm = [-sin(th), cos(th)];
        for ey = eyl, for ep = epl, for dS = dsl
            lapT = mod(ntot, 3);                       % 0, 1, 2 donusumlu
            X = p + ey*nrm;  psi = th + ep + 2*pi*lapT; % psi sarilmamis
            sref = s0 + lapT*r.L + dS;
            [e_x, e_y, e_psi, ~, ~, ~, s_veh, s_proj, idx, lap] = ...
                ref_error(X(1), X(2), psi, 8, 0, 0, sref, 0, 0, 0, r, wf, wb);
            emax = max(emax, abs([e_x - dS, e_y - ey, e_psi - ep]));
            % dikiste s_proj = L - eps, lap = lapT - 1 de dogrudur: s_veh'e bakilir
            nlap = max(nlap, abs(s_veh - (s0 + lapT*r.L)));
            % ayni poz pencereyle (idx_prev ~ 30 dugum geride)
            [~, e_y2, e_psi2, ~, ~, ~, ~, s_proj2] = ref_error(X(1), X(2), psi, 8, 0, 0, ...
                sref, mod(idx - 31, r.N) + 1, lap, s_proj, r, wf, wb);
            ewin = max(ewin, max(abs([e_y2 - e_y, e_psi2 - e_psi, s_proj2 - s_proj])));
            ntot = ntot + 1;
        end, end, end
    end
    res(end+1,:) = chk(sprintf('T4 %s pozlar max|e_x - e_x*| (%d poz)', P.n, ntot), emax(1), 1e-3); %#ok<AGROW>
    res(end+1,:) = chk(sprintf('T4 %s pozlar max|e_y - e_y*|', P.n), emax(2), 1e-3); %#ok<AGROW>
    res(end+1,:) = chk(sprintf('T4 %s pozlar max|e_psi - e_psi*|', P.n), emax(3), 1e-3); %#ok<AGROW>
    res(end+1,:) = chk(sprintf('T4 %s ilk adim max|s_veh - (s* + lap* L)|', P.n), nlap, 1e-3); %#ok<AGROW>
    res(end+1,:) = chk(sprintf('T4 %s pencere = global fark', P.n), ewin, 1e-12); %#ok<AGROW>
end

% yoldan uzak arac (plant testleri (0,0)'dan baslar) ve cember merkezi
far = 0;
for q = 1:2
    for XY = [0 0; 250 250; -1e4 3e4].'
        o = cell(1,15);
        [o{:}] = ref_error(XY(1), XY(2), 0.3, 8, 0.2, 0.1, 5, 0, 0, 0, paths{q}.r, wf, wb);
        far = far + nnz(~isfinite([o{:}]));
    end
end
res(end+1,:) = chk('T4 uzak arac sonlu olmayan cikis sayisi', far, 0.5);

%% ---------------- T4b: dikisten gecen yorungeler ----------------
% Frenet yorungesi: sigma(t), e_y*(t), e_psi*(t) analitik; X, Y, psi, v_x,
% v_y, r bunlardan kurulur (Rajamani 2.5 kinematigi tam saglanir).
ey_f  = @(t) 0.2 + 0.6*sin(0.8*t);    eyd_f = @(t) 0.48*cos(0.8*t);
ep_f  = @(t) 0.08*sin(1.1*t);         epd_f = @(t) 0.088*cos(1.1*t);
trj = {struct('n', 'Kare ileri', 'q', 2, 's0', Lk - 15, 'V',  7,   'T', 11.5, 'lap',  1), ...
       struct('n', 'Kare geri',  'q', 2, 's0', 15,      'V', -6,   'T', 6,    'lap', -1), ...
       struct('n', 'Circle',     'q', 1, 's0', c.L - 10, 'V', 8,   'T', 6,    'lap',  1)};
for z = 1:numel(trj)
    J = trj{z};  P = paths{J.q};  r = P.r;
    t = (0:Ts:J.T).';  nt = numel(t);
    sg  = J.s0 + J.V*t + 0.3*sin(0.9*t);  sgd = J.V + 0.27*cos(0.9*t);
    O = zeros(nt, 15);  ib = zeros(nt,1);  trn = false(nt,1);
    eyT = ey_f(t);  epT = ep_f(t);
    st = [0 0 0];   % idx, lap, s_proj
    for i = 1:nt
        [p, th, ka, dtr] = P.tr(mod(sg(i), r.L));
        T  = [cos(th), sin(th)];  nrm = [-sin(th), cos(th)];
        X  = p + eyT(i)*nrm;
        Pd = T*sgd(i)*(1 - ka*eyT(i)) + eyd_f(t(i))*nrm;
        psi = th + epT(i);
        vb = [cos(psi) sin(psi); -sin(psi) cos(psi)] * Pd.';
        rr = ka*sgd(i) + epd_f(t(i));
        o = cell(1,15);
        [o{:}] = ref_error(X(1), X(2), psi, vb(1), vb(2), rr, sg(i) + 1.0, ...
                           st(1), st(2), st(3), r, wf, wb);
        O(i,:) = [o{:}];
        st = O(i, [9 10 8]);
        [~, ib(i)] = min((r.x - X(1)).^2 + (r.y - X(2)).^2);
        trn(i) = dtr < 1;      % yay-duzluk gecisine 1 m'den yakin
    end
    % O sutunlari: 1 e_x 2 e_y 3 e_psi 4 e_x_dot 5 e_y_dot 6 e_psi_dot 7 s_veh
    %              8 s_proj 9 idx 10 lap 11 v_ref_s 12 v_ref_p 13 kappa_p 14 theta_p 15 s_dot
    ok = ~trn;  in = 2:nt-1;  ok2 = ok(in) & ok(in-1) & ok(in+1);
    cdif = @(q) (q(in+1) - q(in-1)) / (2*Ts);
    res(end+1,:) = chk(['T4 ' J.n ': son tur (beklenen ' num2str(J.lap) ') farki'], abs(O(end,10) - J.lap), 0.5); %#ok<AGROW>
    res(end+1,:) = chk(['T4 ' J.n ': max|e_x - 1| (acilmis s, gecis disi)'], max(abs(O(ok,1) - 1)), 1e-3); %#ok<AGROW>
    res(end+1,:) = chk(['T4 ' J.n ': max|s_veh adimi| / |V Ts| (sureklilik)'], ...
        max(abs(diff(O(:,7)))) / (abs(J.V)*Ts), 1.5); %#ok<AGROW>
    res(end+1,:) = chk(['T4 ' J.n ': pencere idx ~= global idx sayisi'], nnz(O(:,9) ~= ib), 0.5); %#ok<AGROW>
    res(end+1,:) = chk(['T4 ' J.n ': max|e_y - e_y*|'], max(abs(O(:,2) - eyT)), 1e-3); %#ok<AGROW>
    res(end+1,:) = chk(['T4 ' J.n ': max|e_psi - e_psi*| (gecis disi)'], max(abs(O(ok,3) - epT(ok))), 1e-3); %#ok<AGROW>
    dey = cdif(O(:,2));  dep = cdif(O(:,3));
    res(end+1,:) = chk(['T4 ' J.n ': max|e_y_dot - fark(e_y)| (gecis disi)'], ...
        max(abs(O(in(ok2),5) - dey(ok2))), 1e-3); %#ok<AGROW>
    res(end+1,:) = chk(['T4 ' J.n ': max|e_psi_dot - fark(e_psi)| (gecis disi)'], ...
        max(abs(O(in(ok2),6) - dep(ok2))), 1e-3); %#ok<AGROW>
    res(end+1,:) = chk(['T4 ' J.n ': max|s_dot - sigma_dot| (gecis disi)'], max(abs(O(ok,15) - sgd(ok))), 1e-3); %#ok<AGROW>
    % kavsak (+-1 m) bolgesi, |e_y| <= 1: gevsek sinir 1e-2 (theta merkezi farkinin
    % kappa sicramasinda yumusamasi, olculen en buyuk e_x 2.7e-3 m, e_psi 3.4e-3 rad)
    tj = trn & abs(eyT) <= 1;
    res(end+1,:) = chk(['T4 ' J.n ': kavsak max|e_x - 1| (|e_y| <= 1)'], max([0; abs(O(tj,1) - 1)]), 1e-2); %#ok<AGROW>
    res(end+1,:) = chk(['T4 ' J.n ': kavsak max|e_psi - e_psi*| (|e_y| <= 1)'], ...
        max([0; abs(O(tj,3) - epT(tj))]), 1e-2); %#ok<AGROW>
    fprintf(['T4 bilgi %s: gecis disi max|e_y_dot - e_y_dot*| = %.2e m/s; gecislerde (<1 m, %d ornek): ' ...
             'max|e_x - 1| = %.2e m, max|e_psi - e_psi*| = %.2e rad, max|e_y_dot - fark| = %.2e m/s, ' ...
             'max|e_psi_dot - fark| = %.2e rad/s\n'], J.n, max(abs(O(ok,5) - eyd_f(t(ok)))), nnz(trn), ...
        max([0; abs(O(trn,1) - 1)]), max([0; abs(O(trn,3) - epT(trn))]), ...
        max([0; abs(O(in(~ok2),5) - dey(~ok2))]), max([0; abs(O(in(~ok2),6) - dep(~ok2))]));
end

%% ---------------- T5: referans noktasi ilerletme ----------------
r = k;  nmax = ceil(3.05*r.L / (min(r.v)*Ts));
sa = zeros(nmax,1);  sb = sa;  sa(1) = r.s0;  sb(1) = r.s0;
st = [r.s; r.L];  vt = [r.v; r.v(1)];
for i = 1:nmax-1
    sa(i+1) = ref_advance(sa(i), r, Ts);
    sb(i+1) = sb(i) + interp1(st, vt, mod(sb(i), r.L)) * Ts;   % bagimsiz
    if sa(i+1) - r.s0 >= 3*r.L, break; end
end
n3 = i + 1;  sa = sa(1:n3);  sb = sb(1:n3);  tk = (0:n3-1).'*Ts;
res(end+1,:) = chk('T5 ref_advance - bagimsiz ozyineleme max fark [m]', max(abs(sa - sb)), 1e-9);
opt = odeset('RelTol', 1e-10, 'AbsTol', 1e-10);
% MaxStep sart: sabit hizli duzlukte ode45 adimi buyutup virajlari atlar
% (MaxStep'siz tur 38.69 s, dogrusu 39.09 s)
sol = ode45(@(~, s) interp1(st, vt, mod(s, r.L)), [0 tk(end) + 1], r.s0, odeset(opt, 'MaxStep', Ts/2));
sc  = deval(sol, tk).';
rel = max(abs(sa - sc)) / (3*r.L);
res(end+1,:) = chk('T5 ayrik - ode45 max|fark| / 3L', rel, 1e-3);
% 3 tur suresi: ayrik (ilk asim adimi, lineer) vs surekli (deval + fzero)
t3d = tk(n3-1) + (r.s0 + 3*r.L - sa(n3-1)) / (sa(n3) - sa(n3-1)) * Ts;
t3c = fzero(@(tt) deval(sol, tt) - r.s0 - 3*r.L, [tk(end) - 1, tk(end) + 1]);
v1 = vt(1:end-1);  v2 = vt(2:end);  dtt = r.ds ./ v1;  m = abs(v2 - v1) > 1e-12;
dtt(m) = r.ds * log(v2(m)./v1(m)) ./ (v2(m) - v1(m));      % lineer v(s) icin tam ds/v
fprintf(['T5 bilgi: 3 tur suresi ayrik %.6f s, ode45 %.6f s, analitik 3*int(ds/v) %.6f s; ' ...
         'max|s_ayrik - s_ode45| = %.3e m\n'], t3d, t3c, 3*sum(dtt), max(abs(sa - sc)));

%% ---------------- T5: bitis mantigi ----------------
fin = struct('n_laps', 3, 'v_stop', 0.05, 't_hold', 0.5);
% (a) tur olcutu: kusursuz takip, s_veh = s_ref, hiz 8 m/s
kexp = find(sa - r.s0 >= 3*r.L, 1);
[da, de] = run_done(sa, 8*ones(n3,1), r, fin, Ts);
res(end+1,:) = chk('T5 tur olcutu ilk tetik adimi farki', abs(find(da, 1) - kexp), 0.5);
res(end+1,:) = chk('T5 tur olcutu, stop_enable = 0 tetik sayisi', nnz(de), 0.5);
% (b) durma olcutu: 60-69 yavas (kesinti), 70-99 hizli, 100'den sonra yavas
nk = 200;  vv = ones(nk,1);  vv(61:70) = 0.01;  vv(101:end) = 0.01;   % 1 tabanli
[db, dbe] = run_done(r.s0*ones(nk,1), vv, r, fin, Ts);
kb = 101 + round(fin.t_hold/Ts);        % ilk yavas ornek 101, 0.5 s sonra
res(end+1,:) = chk('T5 durma olcutu ilk tetik adimi farki', abs(find(db, 1) - kb), 0.5);
res(end+1,:) = chk('T5 durma olcutu kesintide tetik (adim 61-100)', nnz(db(61:100)), 0.5);
res(end+1,:) = chk('T5 durma olcutu, stop_enable = 0 tetik sayisi', nnz(dbe), 0.5);
fprintf('T5 bilgi: tur tetigi adim %d (t = %.2f s), durma tetigi adim %d (ilk yavas ornekten %.2f s sonra)\n', ...
    find(da, 1), tk(find(da, 1)), find(db, 1), (find(db, 1) - 101)*Ts);

%% ---------------- Ozet ----------------
fprintf('\ntest_error_calc:\n');
fprintf('%-62s %12s %10s %s\n', 'olcut', 'deger', 'esik', 'sonuc');
for i = 1:size(res,1)
    fprintf('%-62s %12.4e %10.2e %s\n', res{i,1}, res{i,2}, res{i,3}, tern(res{i,4}));
end
nf = nnz(~[res{:,4}]);
if nf > 0
    error('test_error_calc: %d olcut KALDI.', nf);
end
fprintf('GECTI (%d olcut)\n', size(res,1));
end

%% ======================================================================
function [d, de] = run_done(sv, vv, r, fin, Ts)
% ref_done'u dizi boyunca calistirir; d: stop_enable = 1, de: stop_enable = 0
n = numel(sv);  d = false(n,1);  de = d;  ns = 0;  nse = 0;
for i = 1:n
    [a, ns]  = ref_done(sv(i), vv(i), 0, ns,  1, r, fin, Ts);
    [b, nse] = ref_done(sv(i), vv(i), 0, nse, 0, r, fin, Ts);
    d(i) = a ~= 0;  de(i) = b ~= 0;
end
end

function [p, th, ka, dtr] = truth_circle(s)
% YuvarlakMap: merkez (250, 250), R = 120, s = 0 aci 0'da, CCW
R = 120;  a = s/R;
p = [250 + R*cos(a), 250 + R*sin(a)];  th = a + pi/2;  ka = 1/R;  dtr = inf;
end

function [p, th, ka, dtr] = truth_rsq(s, so)
% KareMap [60,140]^2, R_c = 8 yuvarlatilmis; sigma = 0 (140, 68) teget
% noktasinda, CCW. Kenar q: 64 m duz + 4 pi m yay. dtr: en yakin gecise yay boyu
Rc = 8;  Ls = 64;  La = Rc*pi/2;  Lq = Ls + La;
sig = mod(s + so, 4*Lq);
q   = min(floor(sig/Lq), 3);  u = sig - q*Lq;
tp  = [140 68; 132 140; 60 132; 68 60];
dr  = [0 1; -1 0; 0 -1; 1 0];
cs  = [132 132; 68 132; 68 68; 132 68];
th0 = pi/2 + q*pi/2;
if u < Ls
    p = tp(q+1,:) + u*dr(q+1,:);  th = th0;  ka = 0;
    dtr = min(u, Ls - u);
else
    th = th0 + (u - Ls)/Rc;
    p  = cs(q+1,:) + Rc*[sin(th), -cos(th)];  ka = 1/Rc;
    dtr = min(u - Ls, Lq - u);
end
th = mod(th + pi, 2*pi) - pi;
end

function [wp, sp] = load_map(file)
% harita betigini calistirir, yalniz onun actigi figurleri kapatir
figs0 = findall(groot, 'Type', 'figure');
run(file);
wp = eval('waypoints');
sp = eval('startPt');
close(setdiff(findall(groot, 'Type', 'figure'), figs0));
end

function row = chk(name, val, tol)
row = {name, val, tol, val < tol};
end

function s = tern(b)
if b, s = 'GECTI'; else, s = 'KALDI'; end
end
