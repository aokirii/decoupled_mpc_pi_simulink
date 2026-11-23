function test_reference_path()
%TEST_REFERENCE_PATH  S2-A yol uretimi ve hiz profili testi (T1, T2, T3).
%   T1 Circle (spline): uzunluk, egrilik (dikis dahil), dikiste ds ve yon
%      sicramasi. Hand (pchip): sentetik seyrek poligonlar A (~90 derece),
%      B (yildiz), C (A + HandFigureMap koprusu ve kapanis tekrari); kendini
%      kesme yok, donus sayisi 1, poligondan sapma, dikis; kapanis tekrarli
%      / tekrarsiz ayni yol.
%   T2 Rectangle (R_c = 8): uzunluk, yay ve duzluk egriligi, teget
%      surekliligi, analitik yuvarlatilmis kareye uzaklik; R_c = 0 rapor.
%   T3 Hiz profili (Kare, Circle, Hand A/B/C): v_max, a_y siniri, elipsli
%      boyuna ivme siniri (dikis dahil), viraj oncesi yavaslama / sonrasi
%      hizlanma (Kare).
%   Olcut asilirsa error. Kullanim (proje kokunden):
%       addpath tests; test_reference_path

root = fileparts(fileparts(mfilename('fullpath')));
oldpath = addpath(root);
cleanup = onCleanup(@() path(oldpath)); %#ok<NASGU>

prm = default_prm();
res = {};   % {ad, deger, esik, gecti}

%% ---------------- T1: Circle ----------------
[wp, sp] = load_map(fullfile(root, 'maps', 'YuvarlakMap.m'));
R = 120;
c = reference_path(wp, sp, prm);
res(end+1,:) = chk('T1 |L-2piR|/2piR', abs(c.L - 2*pi*R)/(2*pi*R), 1e-3);
res(end+1,:) = chk('T1 max|kappa R - 1| (dikis dahil)', max(abs(c.kappa*R - 1)), 1e-3);
[dsea, dmax, hsea, hint] = seam_stats(c);
res(end+1,:) = chk('T1 dikis |d - ds|/ds', dsea, 1e-3);
res(end+1,:) = chk('T1 tum ardisik |d - ds|/ds', dmax, 1e-3);
res(end+1,:) = chk('T1 dikis yon adimi / max ic adim', hsea/hint, 1 + 1e-6);

%% ---------------- T1: Hand (pchip) ----------------
ph = prm;  ph.interp_method = 'pchip';
A  = [10 10; 60 12; 62 40; 30 42; 28 70; 80 72; 85 90; 15 88];        % ~90 derece
tb = pi/2 + (0:9).'*pi/5;  rb = repmat([40; 15], 5, 1);
B  = [50 + rb.*cos(tb), 50 + rb.*sin(tb)];                              % yildiz, uc ~144 derece
dc = hypot(A(end,1) - A(1,1), A(end,2) - A(1,2));  nb = max(5, round(dc/2));
xb = linspace(A(end,1), A(1,1), nb+1).';  yb = linspace(A(end,2), A(1,2), nb+1).';
C  = [A; xb(2:end-1), yb(2:end-1); A(1,:)];                             % HandFigureMap kapanisi
hs = {'A', A; 'B', B; 'C', C};
hr = cell(1,3);
for q = 1:3
    P = hs{q,2};
    r = reference_path(P, P(1,:), ph);  hr{q} = r;
    [dev, nx, tn] = poly_stats(r, P);
    res(end+1,:) = chk(['T1 Hand ' hs{q,1} ' kendini kesme sayisi'], nx, 0.5); %#ok<AGROW>
    res(end+1,:) = chk(['T1 Hand ' hs{q,1} ' |donus sayisi - 1|'], abs(tn - 1), 1e-9); %#ok<AGROW>
    % esik 3 m: olculen en buyuk sapma 2.54 m (B, yildiz uclari 36 derece);
    % spline ayni poligonlarda 18.2 m (A) sapiyor ve B'de ilmek yapiyordu
    res(end+1,:) = chk(['T1 Hand ' hs{q,1} ' poligondan max sapma [m]'], dev, 3); %#ok<AGROW>
    % pchip keskin kosede kiris yay uzunlugundan kisa: ic noktalarda
    % |d - ds|/ds %6-%60'a cikar. Olcut: dikis ic noktalardan kotu degil.
    [dsea, ~, hsea, hint, dint] = seam_stats(r);
    res(end+1,:) = chk(['T1 Hand ' hs{q,1} ' dikis / max ic aralik sapmasi'], dsea/dint, 1 + 1e-6); %#ok<AGROW>
    res(end+1,:) = chk(['T1 Hand ' hs{q,1} ' dikis / max ic yon adimi'], hsea/hint, 1 + 1e-6); %#ok<AGROW>
    fprintf(['T1 bilgi Hand %s (pchip): L = %.3f m, max|kappa| = %.3f 1/m, min v = %.3f m/s, ' ...
             'dikis |d-ds|/ds = %.2e (ic max %.2e)\n'], hs{q,1}, r.L, max(abs(r.kappa)), min(r.v), dsea, dint);
end

% el cizimi sentetik poligon: kapanis tekrarsiz ve tekrarli (pchip)
hp = [20 20; 50 15; 80 25; 85 55; 70 80; 40 85; 15 65; 10 40];
h1 = reference_path(hp, hp(1,:), ph);
h2 = reference_path([hp; hp(1,:)], hp(1,:), ph);
res(end+1,:) = chk('T1 Hand tekrarli/tekrarsiz max|dx,dy|', ...
    max(abs([h1.x - h2.x; h1.y - h2.y])) + abs(h1.L - h2.L), 1e-12);
[dsea, ~, hsea, hint] = seam_stats(h1);
res(end+1,:) = chk('T1 Hand dikis |d - ds|/ds', dsea, 1e-3);
res(end+1,:) = chk('T1 Hand dikis yon adimi / max ic adim', hsea/hint, 1 + 1e-6);
pb = prm;  pb.interp_method = 'linear';   % bilinmeyen yontem error vermeli
try
    reference_path(hp, hp(1,:), pb);  nerr = 0;
catch
    nerr = 1;
end
res(end+1,:) = chk('T1 bilinmeyen interp_method error vermedi', 1 - nerr, 0.5);

%% ---------------- T2: Rectangle, R_c = 8 ----------------
[wk, sk] = load_map(fullfile(root, 'maps', 'KareMap.m'));
Rc = 8;
p8 = prm;  p8.corner_radius = Rc;
k = reference_path(wk, sk, p8);
Lx = 320 - 4*(2 - pi/2)*Rc;
res(end+1,:) = chk('T2 |L - 306.27|/L', abs(k.L - Lx)/Lx, 1e-3);
[inarc, far] = rsq_regions(k.x, k.y, Rc);
res(end+1,:) = chk('T2 yay ici max|kappa Rc - 1| (>1 m)', max(abs(k.kappa(inarc & far)*Rc - 1)), 0.01);
res(end+1,:) = chk('T2 duzluk max|kappa| (>1 m)', max(abs(k.kappa(~inarc & far))), 1e-3);
dth = abs(wrap_pi(diff(k.theta([1:end 1]))));
res(end+1,:) = chk('T2 max yon adimi / (ds/Rc) (teget surekli)', max(dth)/(k.ds/Rc), 1.05);
res(end+1,:) = chk('T2 analitik sekle max uzaklik [m]', max(abs(rsq_sdf(k.x, k.y, Rc))), 1e-4);
fprintf('T2 bilgi: N = %d, ds = %.6f m, yay ici kappa ortalama = %.6f (1/Rc = %.6f)\n', ...
    k.N, k.ds, mean(k.kappa(inarc & far)), 1/Rc);

k0 = reference_path(wk, sk, prm);
fprintf(['T2 bilgi, R_c = 0 (keskin kose, spline): L = %.4f m, max|kappa| = %.3f 1/m, ' ...
         'kareden max tasma = %.4f m, min v = %.3f m/s\n'], ...
    k0.L, max(abs(k0.kappa)), max(rsq_sdf(k0.x, k0.y, 0)), min(k0.v));

%% ---------------- T3: Hiz profili ----------------
for q = {struct('n', 'Kare', 'r', k), struct('n', 'Circle', 'r', c), struct('n', 'Hand A', 'r', hr{1}), ...
         struct('n', 'Hand B', 'r', hr{2}), struct('n', 'Hand C', 'r', hr{3})}
    r = q{1}.r;  nm = q{1}.n;
    v = r.v;  ks = abs(r.kappa_s);  N = r.N;
    res(end+1,:) = chk(['T3 ' nm ' max v - v_max'], max(v) - prm.v_max, 1e-12); %#ok<AGROW>
    fl = v <= prm.v_min + 1e-12;   % taban noktalari
    res(end+1,:) = chk(['T3 ' nm ' max v^2|kappa_s|/ay_max (taban disi)'], ...
        max(v(~fl).^2 .* ks(~fl)) / prm.ay_max, 1 + 1e-9); %#ok<AGROW>
    j  = [2:N 1].';
    a  = (v(j).^2 - v.^2) / (2*r.ds);   % i -> i+1, dikis dahil
    el = @(vi, ki) sqrt(max(0, 1 - (vi.^2.*ki/prm.ay_max).^2));
    ok = ~fl & ~fl(j);
    up = max(a(ok) - prm.ax_max*el(v(ok), ks(ok)));
    lo = max(prm.ax_min*el(v(j(ok)), ks(j(ok))) - a(ok));
    res(end+1,:) = chk(['T3 ' nm ' ivme asimi a - ax_max*elips [m/s^2]'], up, 1e-6); %#ok<AGROW>
    res(end+1,:) = chk(['T3 ' nm ' fren asimi ax_min*elips - a [m/s^2]'], lo, 1e-6); %#ok<AGROW>
    fprintf('T3 bilgi %s: v in [%.4f, %.4f] m/s, taban noktasi %d, max a = %.4f, min a = %.4f m/s^2\n', ...
        nm, min(v), max(v), nnz(fl), max(a), min(a));
end

% Kare: duzluk ortasi -> yay ortasi azalan, yay ortasi -> sonraki duzluk ortasi artan
cs = [132 132; 68 132; 68 68; 132 68];      % yay merkezleri (CCW, ilki sag ust)
sm = [140 100; 100 140; 60 100; 100 60];    % duzluk ortalari (her yaydan once)
v = k.v;  mono = 0;  vmin = inf;
for q = 1:4
    am = cs(q,:) + Rc/sqrt(2)*sign(cs(q,:) - 100);
    i1 = nearest(k, sm(q,:));  i2 = nearest(k, am);  i3 = nearest(k, sm(mod(q,4)+1,:));
    seg1 = mod((i1:i1 + mod(i2 - i1, k.N)) - 1, k.N) + 1;
    seg2 = mod((i2:i2 + mod(i3 - i2, k.N)) - 1, k.N) + 1;
    mono = max([mono, max(diff(v(seg1))), max(-diff(v(seg2)))]);
    vmin = min(vmin, v(i2));
    if q == 1
        ib = seg1(find(v(seg1) < prm.v_max - 1e-9, 1));
        ie = seg2(find(v(seg2) >= prm.v_max - 1e-9, 1));
        fprintf(['T3 bilgi Kare 1. viraj: yavaslama yay ortasindan %.2f m once basliyor, ' ...
                 'v_max''a yay ortasindan %.2f m sonra donuyor\n'], ...
            mod(i2 - ib, k.N)*k.ds, mod(ie - i2, k.N)*k.ds);
    end
end
% esik 1e-4 m/s: yay platosu kenarinda kappa_s'te ~1e-5 bagil dalga var
% (dogru-yay birlesiminde spline'in kappa salinimi, 3 m pencereden artan);
% v = sqrt(ay_max/kappa_s) bunu ~4e-5 m/s olarak tasir.
res(end+1,:) = chk('T3 Kare monotonluk ihlali (yavaslama/hizlanma) [m/s]', mono, 1e-4);
vth = sqrt(prm.ay_max*Rc);
res(end+1,:) = chk('T3 Kare duzluk ortasi v = v_max (|v - v_max|)', abs(max(v(nearest(k, sm))) - prm.v_max) ...
    + abs(min(v(nearest(k, sm))) - prm.v_max), 1e-12);
fprintf('T3 bilgi Kare: virajdaki en dusuk hiz %.4f m/s, sqrt(ay_max Rc) = %.4f m/s (fark %.2e)\n', ...
    vmin, vth, vmin - vth);

%% ---------------- Ozet ----------------
fprintf('\ntest_reference_path:\n');
fprintf('%-58s %12s %10s %s\n', 'olcut', 'deger', 'esik', 'sonuc');
for i = 1:size(res,1)
    fprintf('%-58s %12.4e %10.2e %s\n', res{i,1}, res{i,2}, res{i,3}, tern(res{i,4}));
end
nf = nnz(~[res{:,4}]);
if nf > 0
    error('test_reference_path: %d olcut KALDI.', nf);
end
fprintf('GECTI (%d olcut)\n', size(res,1));
end

%% ======================================================================
function prm = default_prm()
mu  = 0.75;  g = 9.81;
prm = struct('ds_target', 0.1, 'corner_radius', 0, 'interp_method', 'spline', 'v_max', 8, ...
    'ay_max', 0.85*mu*g, 'ax_max', 3, 'ax_min', -6, 'v_min', 2, 'kappa_win', 3);
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

function [dsea, dmax, hsea, hint, dint] = seam_stats(r)
% dikiste (N -> 1) ve tum ardisik noktalarda aralik (dint: dikis haric);
% yon adimi
d    = hypot(r.x([2:end 1]) - r.x, r.y([2:end 1]) - r.y);
dsea = abs(d(end) - r.ds)/r.ds;
dmax = max(abs(d - r.ds))/r.ds;
dint = max(abs(d(1:end-1) - r.ds))/r.ds;
h    = abs(wrap_pi(diff(r.theta([1:end 1]))));
hsea = h(end);
hint = max(h(1:end-1));
end

function [dev, nx, tn] = poly_stats(r, P)
% kapali poligona en buyuk uzaklik, kendini kesme sayisi, donus sayisi
P = P([true; sqrt(sum(diff(P).^2, 2)) > 1e-9], :);
if norm(P(end,:) - P(1,:)) < 1e-6, P(end,:) = []; end
n = size(P,1);  D = inf(size(r.x));
for i = 1:n
    a = P(i,:);  u = P(mod(i,n)+1,:) - a;
    t = min(max(((r.x - a(1))*u(1) + (r.y - a(2))*u(2)) / (u*u.'), 0), 1);
    D = min(D, hypot(r.x - a(1) - t*u(1), r.y - a(2) - t*u(2)));
end
dev = max(D);
X = [r.x r.y; r.x(1) r.y(1)];  m = r.N;  nx = 0;
for i = 1:m-2
    j = (i+2):m;  if i == 1, j = j(j ~= m); end
    p = X(i,:);  d1 = X(i+1,:) - p;  q = X(j,:);  d2 = X(j+1,:) - q;  qp = q - p;
    den = d1(1)*d2(:,2) - d1(2)*d2(:,1);
    t = (qp(:,1).*d2(:,2) - qp(:,2).*d2(:,1)) ./ den;
    u = (qp(:,1)*d1(2) - qp(:,2)*d1(1)) ./ den;
    nx = nx + nnz(abs(den) > 1e-14 & t > 0 & t < 1 & u > 0 & u < 1);
end
tn = sum(wrap_pi(diff(r.theta([1:end 1])))) / (2*pi);
end

function i = nearest(r, P)
i = zeros(size(P,1),1);
for q = 1:size(P,1)
    [~, i(q)] = min((r.x - P(q,1)).^2 + (r.y - P(q,2)).^2);
end
end

function [inarc, far] = rsq_regions(x, y, Rc)
% KareMap: [60,140]^2, koseler Rc ile yuvarlatilmis. inarc = yay bolgesi,
% far = en yakin teget (gecis) noktasina > 1 m
cs = [132 132; 68 132; 68 68; 132 68];
inarc = false(size(x));  dmin = inf(size(x));
for q = 1:4
    sx = sign(cs(q,1) - 100);  sy = sign(cs(q,2) - 100);
    inarc = inarc | ((x - cs(q,1))*sx > 0 & (y - cs(q,2))*sy > 0);
    tp = [cs(q,1) + sx*Rc, cs(q,2); cs(q,1), cs(q,2) + sy*Rc];
    for t = 1:2
        dmin = min(dmin, hypot(x - tp(t,1), y - tp(t,2)));
    end
end
far = dmin > 1;
end

function d = rsq_sdf(x, y, Rc)
% yuvarlatilmis karenin isaretli uzaklik fonksiyonu (+ = disari)
qx = abs(x - 100) - (40 - Rc);
qy = abs(y - 100) - (40 - Rc);
d  = hypot(max(qx,0), max(qy,0)) + min(max(qx,qy), 0) - Rc;
end

function a = wrap_pi(a)
a = mod(a + pi, 2*pi) - pi;
end
