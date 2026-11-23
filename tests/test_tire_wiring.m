function test_tire_wiring()
%TEST_TIRE_WIRING  S1b tam lastik modu kablolama regresyon testi.
%   VehicleDiagram.slx'in Plant'ini tire_mode = 1 ("tam") ile, build_model'den
%   bagimsiz yazilmis bir referans sag tarafin (ref_rhs) ode15s cozumuyle
%   (RelTol = AbsTol = 1e-9) uc senaryoda karsilastirir:
%     FA  surus + fren, delta sinus (v_x 4..13 m/s)
%     FL  kilitlenme, delta = 0 (20 m/s, -15 kN; durusa ve geri sekmeye kadar)
%     A1  direksiyonlu sert fren (20 m/s, delta = 0.05, -15 kN t = 0'da)
%   Karsilastirilan 13 durum: X Y psi v_x v_y r (state) ve logsout'tan
%   omega_f omega_r kappa_f kappa_r alpha_f alpha_r a_x_f.
%   Her durum icin olcekli hata max|x_sim - x_ref| / max(1, max|x_ref|) < 1e-3
%   olmali; asilirsa error. Fark ode4 (1 ms) ayriklastirmasindan gelir.
%
%   PARAMETRELER TESTTE SABITTIR: tire_mode = 1 ve butun govde/lastik
%   parametreleri (v_eps, tau_c dahil) asagidaki tabloda yazilidir ve
%   Simulink.SimulationInput.setVariable ile modele acikca verilir. Ayni
%   degerler referansta da kullanilir. Model_Setup'taki ayar degisiklikleri
%   (ornegin v_eps secimi) bu testi bozmaz; Model_Setup yalniz modelin
%   ihtiyac duyabilecegi diger degiskenler icin cagrilir.
%
%   Referans denklemler: diyagram-simulink-matematik.md, bolum 2.3-2.10
%   (aktuator, tekerlek, gevseme + dusuk hiz sonumu + doyma cekisi, yon
%   duzeltmeli Dugoff, yuk transferi, yone duyarli F_res, v_x sinirsiz).
%   Integrator alt sinirlari (omega >= 0, kappa >= -1) olay (event) ile.
%
%   Kullanim (proje kokunden):  addpath tests; test_tire_wiring
%   Gereken: VehicleDiagram.slx guncel olmali (build_model).

tol  = 1e-3;
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
evalin('base', 'Model_Setup');

% --- sabit parametreler (modele setVariable ile verilir) ---
mv = struct('tire_mode', 1, ...
    'm', 1500, 'Iz', 2250, 'L', 2.7, 'lr', 1.35, 'lf', 1.35, 'Cf', 38000, 'Cr', 38000, ...
    'rho_air', 1.225, 'CdA', 0.66, 'C_rr', 0.015, 'g', 9.81, 'theta_road', 0, ...
    'vx_floor', 1, 'brake_split', 0.6, ...
    'R_e', 0.3, 'I_w', 2.0, 'C_kappa', 19*1500*9.81/2, 'c_x', 2e6, ...
    'sigma_x', (19*1500*9.81/2)/2e6, 'sigma_y', 0.3, 'mu', 0.75, 'h', 0.55, ...
    'tau_ax', 0.1, 'k_low0', 13000, 'v_low', 1, 'tau_c', 0.02, 'v_eps', 0.02, ...
    'Ts_plant', 1e-3, 'vy0_init', 0, 'r0_init', 0, 'X0', 0, 'Y0', 0, 'psi0', 0);
% referans parametreleri (ayni degerler)
P = struct('m',mv.m,'Iz',mv.Iz,'L',mv.L,'lr',mv.lr,'lf',mv.lf,'Cf',mv.Cf,'Cr',mv.Cr, ...
    'rho',mv.rho_air,'CdA',mv.CdA,'Crr',mv.C_rr,'g',mv.g,'th',mv.theta_road,'bs',mv.brake_split, ...
    'Re',mv.R_e,'Iw',mv.I_w,'Ck',mv.C_kappa,'sx',mv.sigma_x,'sy',mv.sigma_y,'mu',mv.mu, ...
    'h',mv.h,'tau',mv.tau_ax,'k0',mv.k_low0,'vlow',mv.v_low,'tauc',mv.tau_c,'veps',mv.v_eps);

% --- senaryo FA: Fx 0 -> 1500 -> -3000 -> 500 (5 ms rampalar), delta = 0.05 sin(0.8 t)
Tend = 14;
Ftab = [0 0; 1 0; 1.005 1500; 5 1500; 5.005 -3000; 9 -3000; 9.005 500; Tend 500];
td = (0:0.02:Tend)'; dtab = [td 0.05*sin(0.8*td)];
cases(1) = struct('name','FA','v0',10,'Ftab',Ftab,'dtab',dtab,'Tend',Tend);
% --- senaryo FL: kilitlenme, delta = 0
Tend = 5;
cases(2) = struct('name','FL','v0',20,'Ftab',[0 0; 0.5 0; 0.505 -15000; Tend -15000], ...
    'dtab',[0 0; Tend 0],'Tend',Tend);
% --- senaryo A1: direksiyonlu sert fren, ikisi de t = 0'da
Tend = 8;
cases(3) = struct('name','A1','v0',20,'Ftab',[0 -15000; Tend -15000], ...
    'dtab',[0 0.05; Tend 0.05],'Tend',Tend);

sn = {'X','Y','psi','vx','vy','r','w_f','w_r','k_f','k_r','a_f','a_r','ax_f'};
lg = {'omega_f','omega_r','kappa_f','kappa_r','alpha_f','alpha_r','a_x_f'};
fv = fieldnames(mv);
err = zeros(numel(cases), numel(sn));
for c = 1:numel(cases)
    K = cases(c);
    in = Simulink.SimulationInput(mdl);
    for iv = 1:numel(fv)
        in = in.setVariable(fv{iv}, mv.(fv{iv}));
    end
    in = in.setVariable('T_end', K.Tend);
    in = in.setVariable('vx0_init', K.v0);
    in = in.setVariable('Fx_in', K.Ftab);
    in = in.setVariable('delta_in', K.dtab);
    out = sim(in);
    t = out.state.Time;
    x = out.state.Data;
    for il = 1:numel(lg)
        x = [x, out.logsout.get(lg{il}).Values.Data]; %#ok<AGROW>
    end
    w0 = K.v0/P.Re;
    Xr = ref_solve(P, K.Ftab, K.dtab, [0 0 0 K.v0 0 0 w0 w0 0 0 0 0 0], t);
    err(c,:) = max(abs(x - Xr), [], 1) ./ max(1, max(abs(Xr), [], 1));
end

fprintf('\ntest_tire_wiring (tire_mode = 1): olcekli hata max|sim - ref| / max(1, max|ref|), esik %.0e\n', tol);
fprintf('%-4s', 'case');
fprintf(' %8s', sn{:});
fprintf('  sonuc\n');
for c = 1:numel(cases)
    fprintf('%-4s', cases(c).name);
    fprintf(' %8.1e', err(c,:));
    if all(err(c,:) < tol), fprintf('  GECTI\n'); else, fprintf('  KALDI\n'); end
end
if any(err(:) >= tol) || any(isnan(err(:)))
    error('test_tire_wiring: esik asildi (max olcekli hata %.3e >= %.0e).', max(err(:)), tol);
end
fprintf('GECTI (max olcekli hata %.3e)\n', max(err(:)));
end

%% ======================================================================
function restore(mdl, wasLoaded, oldpath, oldcfg, cache)
if ~wasLoaded && bdIsLoaded(mdl), close_system(mdl, 0); end
Simulink.fileGenControl('setConfig', 'config', oldcfg);
path(oldpath);
if exist(cache, 'dir'), rmdir(cache, 's'); end
end

%% ======================================================================
function Xr = ref_solve(P, Ftab, dtab, x0, tq)
% ode15s, giris kirilma noktalarinda parca parca; omega >= 0 ve kappa >= -1
% alt sinirlari terminal olayla (sinirdayken turev asagi iterse 0).
opts0 = {'RelTol', 1e-9, 'AbsTol', 1e-9, 'InitialStep', 1e-6};
lb = [-inf -inf -inf -inf -inf -inf 0 0 -1 -1 -inf -inf -inf]';
fi = find(isfinite(lb));
bp = unique([Ftab(:,1); dtab(:,1); tq(1); tq(end)]);
bp = bp(bp >= tq(1) & bp <= tq(end));
Xr = nan(numel(tq), 13); Xr(1,:) = x0(:).'; xc = x0(:);
for ib = 1:numel(bp)-1
    a = bp(ib); b = bp(ib+1); guard = 0;
    while a < b && guard < 1000
        guard = guard + 1;
        atb = isfinite(lb) & xc <= lb + 1e-12; xc(atb) = lb(atb);
        opts = odeset(opts0{:}, 'Events', @(t,x) bound_event(x, lb, fi, atb));
        sol = ode15s(@(t,x) ref_rhs(t, x, P, Ftab, dtab), [a b], xc, opts);
        te = sol.x(end);
        idx = find(tq > a & tq <= te);
        if ~isempty(idx), Xr(idx,:) = deval(sol, tq(idx)).'; end
        xc = sol.y(:,end);
        if ~isempty(sol.xe) && te < b, a = te; else, a = b; end
    end
end
end

function [val, term, dir] = bound_event(x, lb, fi, atb)
% sinirda olmayan bilesen sinira inince; sinirdaki bilesen ayrilip lb+1e-9'a donunce
val = x(fi) - lb(fi);
val(atb(fi)) = val(atb(fi)) - 1e-9;
term = true(size(val)); dir = -ones(size(val));
end

function dx = ref_rhs(t, x, P, Ftab, dtab)
% Bagimsiz tam lastik referansi. x = [X Y psi vx vy r wf wr kf kr af ar a_f]
% (af, ar = tan(alpha) durumlari, a_f = filtrelenmis a_x)
Fx = lininterp_hold(Ftab, t); d = lininterp_hold(dtab, t);
psi = x(3); vx = x(4); vy = x(5); r = x(6);
w = x(7:8); kap = x(9:10); al = x(11:12); af = x(13);
% aktuator
if Fx >= 0, T = [0; Fx*P.Re]; else, T = [P.bs*Fx*P.Re; (1 - P.bs)*Fx*P.Re]; end
% teker cercevesi hizlari
vwx = [vx*cos(d) + (vy + P.lf*r)*sin(d); vx];
vwy = [-vx*sin(d) + (vy + P.lf*r)*cos(d); vy - P.lr*r];
% yuk transferi
Fz = [max((P.m*P.g*P.lr - P.m*af*P.h)/P.L, 0); max((P.m*P.g*P.lf + P.m*af*P.h)/P.L, 0)];
Ca = [P.Cf; P.Cr];
Fxw = zeros(2,1); Fyw = zeros(2,1); kd = zeros(2,1); ad = zeros(2,1);
for i = 1:2
    V = abs(vwx(i));
    if V < P.vlow, wt = 0.5*(1 + cos(pi*V/P.vlow)); else, wt = 0; end
    kv = P.k0*wt;
    ke = max(kap(i) + kv/P.Ck*(P.Re*w(i) - vwx(i)), -1);
    te = al(i) - kv/Ca(i)*vwy(i);
    [Fxw(i), Fyw(i)] = dugoff_dir(ke, te, Fz(i), P.Ck, Ca(i), P.mu);
    ks = P.mu*Fz(i)/P.Ck; as = P.mu*Fz(i)/Ca(i);
    kd(i) = (P.Re*w(i) - vwx(i) - V*kap(i))/P.sx - wt/P.tauc*(kap(i) - min(max(kap(i), -ks), ks));
    ad(i) = (-vwy(i) - V*al(i))/P.sy     - wt/P.tauc*(al(i)  - min(max(al(i),  -as), as));
end
wd = (T - P.Re*Fxw)/P.Iw;
Fres = 0.5*P.rho*P.CdA*vx*abs(vx) + P.Crr*P.m*P.g*cos(P.th)*tanh(vx/P.veps) + P.m*P.g*sin(P.th);
Fxf = Fxw(1); Fyf = Fyw(1); Fxr = Fxw(2); Fyr = Fyw(2);
ax  = (Fxf*cos(d) - Fyf*sin(d) + Fxr - Fres)/P.m;
dvx = ax + vy*r;
dvy = (Fyf*cos(d) + Fxf*sin(d) + Fyr)/P.m - vx*r;
dr  = (P.lf*(Fyf*cos(d) + Fxf*sin(d)) - P.lr*Fyr)/P.Iz;
daf = (ax - af)/P.tau;
dx = [vx*cos(psi) - vy*sin(psi); vx*sin(psi) + vy*cos(psi); r; dvx; dvy; dr; wd; kd; ad; daf];
% doymus integrator: sinirdayken asagi iten turev 0
lb = [-inf -inf -inf -inf -inf -inf 0 0 -1 -1 -inf -inf -inf]';
dx(x(:) <= lb + 1e-12 & dx < 0 & isfinite(lb)) = 0;
end

function [Fx, Fy] = dugoff_dir(kk, tt, Fz, Ck, Ca, mu)
% Dugoff; lambda < 1 dalinda yon (Ck k, Ca t)/S ile (k, t)/n karisimi
S = sqrt((Ck*kk)^2 + (Ca*tt)^2); n = sqrt(kk^2 + tt^2); u = 1 + kk;
if 2*S <= mu*Fz*u
    if u == 0, Fx = 0; Fy = 0; else, Fx = Ck*kk/u; Fy = Ca*tt/u; end
elseif n == 0
    Fx = 0; Fy = 0;
else
    lam = mu*Fz*u/(2*S); Fm = mu*Fz*(2 - lam)/2;
    e = lam*[Ck*kk, Ca*tt]/S + (1 - lam)*[kk, tt]/n;
    e = e/norm(e); Fx = Fm*e(1); Fy = Fm*e(2);
end
end

function u = lininterp_hold(tab, t)
if t <= tab(1,1), u = tab(1,2); elseif t >= tab(end,1), u = tab(end,2);
else, u = interp1(tab(:,1), tab(:,2), t, 'linear'); end
end
