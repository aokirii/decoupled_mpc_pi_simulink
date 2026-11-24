function test_plant_wiring()
%TEST_PLANT_WIRING  Plant kablolama regresyon testi (sade plant, belge 5).
%   VehicleDiagram.slx'in Plant'ini, build_model'den bagimsiz yazilmis bir
%   referans sag tarafin (ref_rhs: govde a_x'inden yuk transferi, dagitim,
%   kirpma, yanal Dugoff) ode45 cozumuyle (RelTol = AbsTol = 1e-10) dort
%   senaryoda karsilastirir. Referansta a_x ortuktur (F_z <-> kuvvetler <->
%   a_x); her sag taraf degerlendirmesinde sabit nokta iterasyonuyla cozulur.
%   Plant ayni dongudeki a_x'i Memory a_x ile bir adim (Ts_plant) gecikmeli
%   kullanir; bu O(Ts_plant) bir farktir ve doymus kayma senaryolarinda (C,
%   D) olcekli hatayi ~3e-3'e cikarir. Bu yuzden esik senaryo basina:
%   A, B (doymasiz, ~1e-8) 1e-6; C, D 1e-2. C, D ayrica
%   Ts_plant/2 ile yeniden kosulur: hata orani 2 +- 0.2 olmali (birinci
%   mertebe yakinsama: fark yalniz gecikmeden gelir, kablolama hatasi
%   olsaydi adimla kuculmezdi). Olcut: max|x_sim - x_ref| / max(1,
%   max|x_ref|) < tol; asilirsa error.
%   Ayrica loglardan dal kapsamasi (tum senaryolar toplami, her aks): Dugoff
%   dogrusal kolu, doymus kol (alpha > 0 ve alpha < 0), surtunme cemberinin
%   yanal tavani dusurmesi ve F_x kirpmasi en az 10 ornekte (10 ms).
%   Kullanim (proje kokunden):  addpath tests; test_plant_wiring
%   Gereken: VehicleDiagram.slx guncel olmali (build_model).

tol  = [1e-6 1e-6 1e-2 1e-2];   % senaryo basina (A B C D)
root = fileparts(fileparts(mfilename('fullpath')));
mdl  = 'VehicleDiagram';

% path ve Simulink onbellegi gecici; sonda eski haline doner
wasLoaded = bdIsLoaded(mdl);
oldpath = addpath(root);
cache   = tempname;
oldcfg  = Simulink.fileGenControl('getConfig');
Simulink.fileGenControl('set', 'CacheFolder', cache, 'CodeGenFolder', cache, ...
    'createDir', true);
cleanup = onCleanup(@() restore(mdl, wasLoaded, oldpath, oldcfg, cache)); %#ok<NASGU>

assert(strcmp(which([mdl '.slx']), fullfile(root, [mdl '.slx'])), ...
    'Path''te baska bir %s.slx var: %s', mdl, which([mdl '.slx']));
evalin('base', 'map_name = ''Rectangle''; Model_Setup');

% referans parametreleri (Model_Setup'tan bagimsiz yazildi)
P = struct('m',1500,'Iz',2250,'L',2.7,'lr',1.35,'lf',2.7-1.35,'Cf',38000,'Cr',38000, ...
  'rho',1.225,'CdA',0.66,'Crr',0.015,'g',9.81,'th',0,'vxf',1,'bs',0.6, ...
  'mu',0.75,'h',0.55);

% --- senaryo A: Fx 0 -> 800 -> -1500 (5 ms rampalar), delta = 0.05 sin(0.5 t)
Tend = 14;
Ftab = [0 0; 2 0; 2.005 800; 6 800; 6.005 -1500; 10 -1500; 10.005 0; Tend 0];
td = (0:0.01:Tend)'; dtab = [td 0.05*sin(0.5*td)];
cases(1) = struct('name','A','v0',8,'Ftab',Ftab,'dtab',dtab,'Tend',Tend,'conv',false);
% --- senaryo B: dusuk hiz, vx vx_floor (=1) altina iner: fren -600, delta rampasi 0 -> 0.1
Tend = 6;
Ftab = [0 0; 0.5 0; 0.505 -600; Tend -600];
dtab = [0 0; 0.2 0; 0.25 0.1; Tend 0.1];
cases(2) = struct('name','B','v0',3,'Ftab',Ftab,'dtab',dtab,'Tend',Tend,'conv',false);
% --- senaryo C: buyuk delta, isaret degistiren ivmelenme/fren
Tend = 10;
Ftab = [0 1500; 3 1500; 3.002 -3000; 5 -3000; 5.002 1000; Tend 1000];
td = (0:0.005:Tend)'; dtab = [td 0.15*sin(1.3*td).*(td<8)];
cases(3) = struct('name','C','v0',15,'Ftab',Ftab,'dtab',dtab,'Tend',Tend,'conv',true);
% --- senaryo D: doyma; 15 m/s, delta 0 -> -0.15 basamak (0.5 s, 5 ms; saga,
%     alpha < 0), sonra F_x 0 -> -14 kN rampa (2-3 s) ve tutma (iki aks da
%     mu F_z'de kirpilir). alpha > 0 doymus kol C'den gelir.
Tend = 4.5;
Ftab = [0 0; 2 0; 3 -14000; Tend -14000];
dtab = [0 0; 0.5 0; 0.505 -0.15; Tend -0.15];
cases(4) = struct('name','D','v0',15,'Ftab',Ftab,'dtab',dtab,'Tend',Tend,'conv',true);

names = {'X','Y','psi','vx','vy','r'};
err = zeros(numel(cases), 6);
tcmp = zeros(numel(cases), 1);
ratio = nan(numel(cases), 1);
for c = 1:numel(cases)
    K = cases(c);
    in = Simulink.SimulationInput(mdl);
    in = in.setVariable('T_end', K.Tend);
    in = in.setVariable('vx0_init', K.v0);
    in = in.setVariable('Fx_in', K.Ftab);
    in = in.setVariable('delta_in', K.dtab);
    % S2 etkisiz: baslangic konumu referans ile ayni (Model_Setup artik yolun
    % basini verir), bitis mantigi kapali (B'de arac durur)
    in = in.setVariable('X0', 0);
    in = in.setVariable('Y0', 0);
    in = in.setVariable('psi0', 0);
    in = in.setVariable('stop_enable', 0);
    in = in.setVariable('ctrl_enable', 0);   % S3 MPC devre disi: test girisleri plant'a
    out = sim(in);
    [err(c,:), tcmp(c)] = scaled_err(out, P, K);
    covc(:,:,c) = coverage(out, P); %#ok<AGROW>
    if K.conv   % Ts_plant/2: Memory a_x gecikmesinden gelen fark yarilanmali
        o2 = sim(in.setVariable('Ts_plant', 5e-4));
        ratio(c) = max(err(c,:)) / max(scaled_err(o2, P, K));
    end
end
cov = sum(covc(1:2,:,:), 3);  fy_err = max(covc(3,1:2,:), [], 'all');

fprintf('\ntest_plant_wiring: olcekli hata max|sim - ref| / max(1, max|ref|), esik A-D %s\n', mat2str(tol));
fprintf('%-4s %8s', 'case', 't_cmp');
fprintf(' %10s', names{:});
fprintf('\n');
for c = 1:numel(cases)
    fprintf('%-4s %8.3f', cases(c).name, tcmp(c));
    fprintf(' %10.3e', err(c,:));
    if cases(c).conv, fprintf('   oran(Ts / Ts/2) %.3f', ratio(c)); end
    fprintf('\n');
end
fprintf('\nDal kapsamasi (ornek sayisi, 1 ms, A-D toplami; esik >= 10):\n');
fprintf('%-6s %9s %9s %9s %9s %9s\n', 'aks', 'dogrusal', 'doymus+', 'doymus-', 'cember', 'kirpma');
fprintf('%-6s %9d %9d %9d %9d %9d\n', 'on',   cov(1,:));
fprintf('%-6s %9d %9d %9d %9d %9d\n', 'arka', cov(2,:));
for c = 1:numel(cases)
    fprintf('  %s: on %s, arka %s\n', cases(c).name, mat2str(covc(1,:,c)), mat2str(covc(2,:,c)));
end
fprintf('max|F_y - Dugoff(log)| = %.3e N\n', fy_err);
if any(max(err, [], 2) >= tol(:))
    error('test_plant_wiring: esik asildi (senaryo basina max olcekli hata %s, esik %s).', ...
        mat2str(max(err, [], 2).', 3), mat2str(tol));
end
if any(abs(ratio([cases.conv]) - 2) > 0.2)
    error('test_plant_wiring: Ts_plant/2 ile hata yarilanmadi (oran %s).', mat2str(ratio', 3));
end
if any(cov(:) < 10)
    error('test_plant_wiring: tetiklenmeyen dal var.');
end
if fy_err > 1e-6
    error('test_plant_wiring: loglanan F_y Dugoff(log) ile tutarsiz.');
end
fprintf('GECTI (max olcekli hata %.3e; dal kapsamasi tam)\n', max(err(:)));
end

%% ======================================================================
function restore(mdl, wasLoaded, oldpath, oldcfg, cache)
if ~wasLoaded && bdIsLoaded(mdl), close_system(mdl, 0); end
Simulink.fileGenControl('setConfig', 'config', oldcfg);
path(oldpath);
if exist(cache, 'dir'), rmdir(cache, 's'); end
end

%% ======================================================================
function [e, tc] = scaled_err(out, P, K)
t = out.state.Time;  x = out.state.Data;
Xr = ref_solve(P, K.Ftab, K.dtab, [0 0 0 K.v0 0 0], t);
valid = Xr(:,4) > 0.05;   % B: v_x -> 0 sinirindan hemen once kesilir
e  = max(abs(x(valid,:) - Xr(valid,:)), [], 1) ./ max(1, max(abs(Xr(valid,:)), [], 1));
tc = t(find(valid, 1, 'last'));
end

function Xr = ref_solve(P, Ftab, dtab, x0, tq)
% ode45 piecewise across all input breakpoints (inputs smooth within segments)
opts = odeset('RelTol',1e-10,'AbsTol',1e-10);
bp = unique([Ftab(:,1); dtab(:,1); tq(1); tq(end)]);
bp = bp(bp >= tq(1) & bp <= tq(end));
Xr = zeros(numel(tq), 6); Xr(1,:) = x0(:).'; xc = x0(:);
for k = 1:numel(bp)-1
  a = bp(k); b = bp(k+1);
  idx = find(tq > a & tq <= b);
  sol = ode45(@(t,x) ref_rhs(t,x,P,Ftab,dtab), [a b], xc, opts);
  if ~isempty(idx), Xr(idx,:) = deval(sol, tq(idx)).'; end
  xc = sol.y(:,end);
end
end

function dx = ref_rhs(t, x, P, Ftab, dtab)
% Independent reference, x = [X Y psi vx vy r]; inputs piecewise linear tables [t u]
Fx = lininterp_hold(Ftab, t); d = lininterp_hold(dtab, t);
X=x(1); Y=x(2); psi=x(3); vx=x(4); vy=x(5); r=x(6); %#ok<NASGU>
% a_x (body longitudinal acceleration) is implicit: loads depend on a_x, forces
% on loads, a_x on forces. Fixed-point iteration (contraction ~ mu h / L).
ax = 0;
for it = 1:200
    [Fxf, Fyf, Fxr, Fyr] = axle_forces(P, Fx, d, vx, vy, r, ax);
    Fres = 0.5*P.rho*P.CdA*vx^2 + P.Crr*P.m*P.g*cos(P.th) + P.m*P.g*sin(P.th);
    dvx = (Fxf*cos(d) - Fyf*sin(d) + Fxr - Fres)/P.m + vy*r;
    axn = dvx - vy*r;
    if abs(axn - ax) < 1e-13, break; end
    ax = axn;
end
assert(it < 200, 'a_x sabit nokta iterasyonu yakinsamadi');
if vx <= 0 && dvx < 0, dvx = 0; end
dvy = (Fyf*cos(d) + Fxf*sin(d) + Fyr)/P.m - vx*r;
dr  = (P.lf*(Fyf*cos(d) + Fxf*sin(d)) - P.lr*Fyr)/P.Iz;
dx = [vx*cos(psi) - vy*sin(psi); vx*sin(psi) + vy*cos(psi); r; dvx; dvy; dr];
end

function [Fxf, Fyf, Fxr, Fyr] = axle_forces(P, Fx, d, vx, vy, r, ax)
Fzf = max(0, P.m*(P.g*P.lr - ax*P.h)/P.L);
Fzr = max(0, P.m*(P.g*P.lf + ax*P.h)/P.L);
if Fx >= 0, Fxf = 0; Fxr = Fx; else, Fxf = P.bs*Fx; Fxr = (1-P.bs)*Fx; end
Fxf = sign(Fxf)*min(abs(Fxf), P.mu*Fzf);
Fxr = sign(Fxr)*min(abs(Fxr), P.mu*Fzr);
vh = max(vx, P.vxf);
af = d - atan((vy + P.lf*r)/vh);
ar = -atan((vy - P.lr*r)/vh);
Fyf = dugoff_ref(af, P.Cf, sqrt(max(0, (P.mu*Fzf)^2 - Fxf^2)));
Fyr = dugoff_ref(ar, P.Cr, sqrt(max(0, (P.mu*Fzr)^2 - Fxr^2)));
end

function F = dugoff_ref(a, C, Fm)
% Lateral Dugoff written with lambda = Fm / (2 C |tan a|) (Rajamani ch. 13,
% kappa = 0): F = C tan a * f(lambda), f = lambda (2 - lambda) for lambda < 1,
% else 1. tan a = 0 -> F = 0.
t = tan(a);
if t == 0, F = 0; return; end
lam = Fm/(2*C*abs(t));
if lam < 1, f = lam*(2 - lam); else, f = 1; end
F = C*t*f;
end

function cov = coverage(out, P)
% Branch coverage from logged signals. Rows 1-2 (front / rear): sample counts
% [linear (2S <= Fmax, |a| > 1e-3), saturated a > 0, saturated a < 0,
% saturated at a reduced cap (|F_x| > 0.05 mu F_z), F_x clipped];
% row 3 (cols 1-2): max|F_y - Dugoff(logged inputs)| per axle.
g  = @(n) out.logsout.get(n).Values.Data;
x  = out.state.Data;  u = out.u_in.Data;
Fz = [g('F_zf') g('F_zr')];  Fx = [g('F_xf') g('F_xr')];  Fy = [g('F_yf') g('F_yr')];
assert(size(Fz,1) == size(x,1) && size(u,1) == size(x,1), 'log uzunluklari farkli');
vh = max(x(:,4), P.vxf);
al = [u(:,2) - atan((x(:,5) + P.lf*x(:,6))./vh), -atan((x(:,5) - P.lr*x(:,6))./vh)];
Fc = [P.bs*min(u(:,1), 0), max(u(:,1), 0) + (1-P.bs)*min(u(:,1), 0)];   % commanded split
C  = [P.Cf P.Cr];  cov = zeros(3, 5);
for i = 1:2
    cap = P.mu*Fz(:,i);
    Fym = sqrt(max(0, cap.^2 - Fx(:,i).^2));
    S   = C(i)*abs(tan(al(:,i)));
    sat = 2*S > Fym & Fym > 0.05*cap;                         % non-trivial saturation
    cov(i,1) = nnz(2*S <= Fym & abs(al(:,i)) > 1e-3);
    cov(i,2) = nnz(sat & al(:,i) > 0);
    cov(i,3) = nnz(sat & al(:,i) < 0);
    cov(i,4) = nnz(sat & abs(Fx(:,i)) > 0.05*cap);             % saturated at a reduced cap
    cov(i,5) = nnz(abs(Fc(:,i)) > cap & abs(abs(Fx(:,i)) - cap) < 1e-9*cap);
    Fr = arrayfun(@(a, f) dugoff_ref(a, C(i), f), al(:,i), Fym);
    cov(3,i) = max(abs(Fy(:,i) - Fr));
end
end

function u = lininterp_hold(tab, t)
if t <= tab(1,1), u = tab(1,2); elseif t >= tab(end,1), u = tab(end,2);
else, u = interp1(tab(:,1), tab(:,2), t, 'linear'); end
end
