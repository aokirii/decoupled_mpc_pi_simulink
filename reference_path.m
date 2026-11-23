function ref = reference_path(waypoints, startPt, prm)
%REFERENCE_PATH  Kapali, periyodik referans yol ve B1 hiz profili (S2).
%   ref = reference_path(waypoints, startPt, prm)
%   waypoints : Nx2 harita noktalari (maps/*.m). Yol kapali kabul edilir;
%               sondaki kapanis tekrari (son = ilk) atilir, kapanis
%               segmenti yola dahildir.
%   startPt   : 1x2 baslangic noktasi (idx0, s0, theta0 icin).
%   prm       : ds_target, corner_radius (0 = kapali), interp_method
%               ('spline' | 'pchip', zorunlu), v_max, ay_max, ax_max,
%               ax_min, v_min, kappa_win.
%   Yontem: (1) corner_radius > 0 ise keskin koseler analitik yayla
%   yuvarlanir; (2) noktalar iki uctan sarmalanip interp_method ile
%   interpole edilir (dikis duzgun); (3) gercek yay uzunluguna gore
%   esit ds ile yeniden orneklenir, [0, L) araligi alinir; (4) kappa
%   periyodik merkezi farkla (lonlat gradient formu); (5) lonlat B1 hiz
%   profili (MPC_Setup.m 84-115) aynen.
%   Yontem haritaya gore secilir: Circle, Rectangle 'spline' (pchip
%   cemberde |kappa*R - 1| = 0.82 verir, spline 3e-6); Hand 'pchip'
%   (spline seyrek el cizimi poligonda 18 m sapar, yildizda ilmek yapar).
%   Ayrinti: diyagram-simulink-matematik.md, 3. bolum.

%% --- Noktalari temizle, kapanis tekrarini at ---
wp = waypoints(all(isfinite(waypoints), 2), :);
wp = wp([true; sqrt(sum(diff(wp).^2, 2)) > 1e-9], :);
if norm(wp(end,:) - wp(1,:)) < 1e-6, wp(end,:) = []; end
if size(wp,1) < 3, error('reference_path: kapali yol icin en az 3 nokta gerekli.'); end

%% --- Kose yuvarlama ---
if prm.corner_radius > 0
    wp = round_corners(wp, prm.corner_radius, prm.ds_target/4);
end

%% --- Periyodik interpolasyon (sarmali uzatma) ---
n  = size(wp,1);
d  = sqrt(sum((wp([2:n 1],:) - wp).^2, 2));   % kapanis segmenti dahil
sc = [0; cumsum(d)];
Lc = sc(end);  sc(end) = [];
k  = min(20, n);
se = [sc(n-k+1:n) - Lc; sc; sc(1:k) + Lc];
switch prm.interp_method
    case 'spline', pp = spline(se.', wp([n-k+1:n, 1:n, 1:k], :).');
    case 'pchip',  pp = pchip(se.', wp([n-k+1:n, 1:n, 1:k], :).');
    otherwise, error('reference_path: interp_method ''spline'' ya da ''pchip'' olmali.');
end

%% --- Gercek yay uzunluguyla esit ds ---
sf = linspace(0, Lc, 4*max(round(Lc/prm.ds_target), 3) + 1);
Pf = ppval(pp, sf);
sa = [0, cumsum(sqrt(sum(diff(Pf, 1, 2).^2, 1)))];
L  = sa(end);
N  = max(round(L/prm.ds_target), 3);
ds = L/N;
s  = (0:N-1).' * ds;
P  = ppval(pp, interp1(sa, sf, s.'));
x  = P(1,:).';  y = P(2,:).';

%% --- Yon ve egrilik (periyodik merkezi fark) ---
cd  = @(q) (q([2:N 1]) - q([N 1:N-1])) / (2*ds);
xp  = cd(x);   yp  = cd(y);
xpp = cd(xp);  ypp = cd(yp);
kappa = (xp.*ypp - yp.*xpp) ./ (xp.^2 + yp.^2).^1.5;
theta = atan2(yp, xp);

[~, idx0] = min((x - startPt(1)).^2 + (y - startPt(2)).^2);

%% --- B1 hiz profili (lonlat MPC_Setup.m 84-115) ---
ay_max  = prm.ay_max;
kappa_h = wrap_pi(diff(theta([1:end 1]))) / ds;
half_w  = min(round(prm.kappa_win/ds/2), floor((N-1)/2));
kappa_s = movmean([kappa_h(end-half_w+1:end); kappa_h; kappa_h(1:half_w)], ...
                  2*half_w+1, 'Endpoints', 'discard');

v = min(prm.v_max, sqrt(ay_max ./ max(abs(kappa_s), eps)));
a_avail = @(vi, ki, a_lim) a_lim * sqrt(max(0, 1 - (vi^2*abs(ki)/ay_max)^2));
for pass = 1:20
    v_prev = v;
    for i = 1:N
        j = mod(i, N) + 1;
        v(j) = min(v(j), sqrt(v(i)^2 + 2*a_avail(v(i), kappa_s(i), prm.ax_max)*ds));
    end
    for i = N:-1:1
        j = mod(i, N) + 1;
        v(i) = min(v(i), sqrt(v(j)^2 + 2*a_avail(v(j), kappa_s(j), -prm.ax_min)*ds));
    end
    if max(abs(v - v_prev)) < 1e-9, break; end
end
v = max(v, min(prm.v_min, prm.v_max));

ref = struct('x', x, 'y', y, 'theta', theta, 'kappa', kappa, ...
             'kappa_s', kappa_s, 'v', v, 's', s, 'N', N, 'ds', ds, 'L', L, ...
             'idx0', idx0, 's0', s(idx0), 'theta0', theta(idx0));
end

%% ======================================================================
function P = round_corners(wp, Rc, h)
% Dogrusal parcali kapali yolda her koseye Rc yaricapli teget yay koyar.
% Dogru = ardisik yonu ayni (1e-6 rad) segmentler. Iki uzun dogru arasinda
% kalan, max(Rc, 2 x medyan segment uzunlugu)'ndan kisa tek segment (ornek
% noktasinin koseye dusmedigi kesik kiris) atilir. Kose = komsu iki dogrunun
% kesisimi. Sonuc h aralikli analitik ornekler; ilk nokta wp(1)'e en yakin ornek.
n   = size(wp,1);
D   = wp([2:n 1],:) - wp;                  % segment i: wp(i) -> wp(i+1)
phi = atan2(D(:,2), D(:,1));
len = sqrt(sum(D.^2, 2));
brk = abs(wrap_pi(phi([2:n 1]) - phi)) > 1e-6;   % i'den sonra yon degisir
if ~any(brk), error('reference_path: yonu degismeyen yol kapali olamaz.'); end

% segmentleri, bir kirilmadan hemen sonra baslayarak dogrulara grupla
i0  = find(brk, 1);
ord = mod(i0 + (0:n-1), n) + 1;
run = cumsum([1, brk(ord(1:end-1)).']);   % ord(k) segmentinin dogru numarasi
nr  = run(end);
A = zeros(nr,2); B = zeros(nr,2); cnt = zeros(nr,1); rl = zeros(nr,1);
for r = 1:nr
    sg = ord(run == r);
    A(r,:) = wp(sg(1),:);  B(r,:) = wp(mod(sg(end), n) + 1, :);
    cnt(r) = numel(sg);    rl(r) = sum(len(sg));
end
% kesik kiris <= bir ornek araligi; esik max(Rc, 2 x medyan segment) R_c
% ornek araligindan kucukken de kirisi yakalar (ornek: Kare'de medyan 0.21 m,
% R_c >= 0.43 m'de esik = Rc)
cut  = cnt == 1 & rl < max(Rc, 2*median(len)) & cnt([nr 1:nr-1]) >= 2 & cnt([2:nr 1]) >= 2;
A = A(~cut,:);  B = B(~cut,:);  nl = size(A,1);
if nl < 3, error('reference_path: kose yuvarlama icin en az 3 dogru gerekli.'); end
U = (B - A) ./ sqrt(sum((B - A).^2, 2));

% koseler: dogru k ile k+1'in kesisimi, donus acisi, teget uzunlugu
C = zeros(nl,2);  dlt = zeros(nl,1);
for c = 1:nl
    c2 = mod(c, nl) + 1;
    ab = [U(c,:).', -U(c2,:).'] \ (A(c2,:) - A(c,:)).';
    C(c,:) = A(c,:) + ab(1)*U(c,:);
    dlt(c) = wrap_pi(atan2(U(c2,2), U(c2,1)) - atan2(U(c,2), U(c,1)));
end
if any(abs(dlt) > pi - 1e-3), error('reference_path: geri donuslu kose yuvarlanamaz.'); end
tl = Rc * tan(abs(dlt)/2);

% dogru k: kose k-1'den kose k'ya; ardindan kose k'daki yay
P = zeros(0,2);
for c = 1:nl
    cp  = mod(c-2, nl) + 1;
    Ps  = C(cp,:) + tl(cp)*U(c,:);
    Pe  = C(c,:)  - tl(c)*U(c,:);
    seg = (Pe - Ps) * U(c,:).';
    if seg < -1e-9
        error('reference_path: %d. dogru R_c = %g icin kisa (%.3f m eksik).', c, Rc, -seg);
    end
    m = ceil(seg/h);
    if m > 0, P = [P; Ps + ((0:m-1).'/m) * (Pe - Ps)]; end %#ok<AGROW>
    nrm = [-U(c,2), U(c,1)];
    O   = Pe + sign(dlt(c))*Rc*nrm;
    a0  = atan2(Pe(2) - O(2), Pe(1) - O(1));
    m   = max(ceil(Rc*abs(dlt(c))/h), 1);
    a   = a0 + dlt(c)*(0:m-1).'/m;
    P   = [P; O + Rc*[cos(a), sin(a)]]; %#ok<AGROW>
end
[~, i1] = min(sum((P - wp(1,:)).^2, 2));
P = circshift(P, 1 - i1);
end

%% ======================================================================
function a = wrap_pi(a)
a = mod(a + pi, 2*pi) - pi;
end
