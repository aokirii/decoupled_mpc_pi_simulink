function test_plant_wiring()
%TEST_PLANT_WIRING  S1 kablolama regresyon testi (kriter 3).
%   VehicleDiagram.slx'in Plant'ini, build_model'den bagimsiz yazilmis bir
%   referans sag tarafin (ref_rhs) ode45 cozumuyle (RelTol = AbsTol = 1e-10)
%   uc senaryoda karsilastirir. Her durum icin olcekli hata
%   max|x_sim - x_ref| / max(1, max|x_ref|) < 1e-4 olmali; asilirsa error.
%   Kullanim (proje kokunden):  addpath tests; test_plant_wiring
%   Gereken: VehicleDiagram.slx guncel olmali (build_model).

tol  = 1e-4;
root = fileparts(fileparts(mfilename('fullpath')));
mdl  = 'VehicleDiagram';

% path ve Simulink onbellegi gecici; sonda eski haline doner
oldpath = addpath(root);
cache   = tempname;
oldcfg  = Simulink.fileGenControl('getConfig');
Simulink.fileGenControl('set', 'CacheFolder', cache, 'CodeGenFolder', cache, ...
    'createDir', true);
cleanup = onCleanup(@() restore(mdl, oldpath, oldcfg, cache)); %#ok<NASGU>

assert(strcmp(which([mdl '.slx']), fullfile(root, [mdl '.slx'])), ...
    'Path''te baska bir %s.slx var: %s', mdl, which([mdl '.slx']));
evalin('base', 'Model_Setup');

% referans parametreleri (Model_Setup'tan bagimsiz yazildi)
P = struct('m',1500,'Iz',2250,'L',2.7,'lr',1.35,'lf',2.7-1.35,'Cf',38000,'Cr',38000, ...
  'rho',1.225,'CdA',0.66,'Crr',0.015,'g',9.81,'th',0,'vxf',1,'bs',0.6);

% --- senaryo A: Fx 0 -> 800 -> -1500 (5 ms rampalar), delta = 0.05 sin(0.5 t)
Tend = 14;
Ftab = [0 0; 2 0; 2.005 800; 6 800; 6.005 -1500; 10 -1500; 10.005 0; Tend 0];
td = (0:0.01:Tend)'; dtab = [td 0.05*sin(0.5*td)];
cases(1) = struct('name','A','v0',8,'Ftab',Ftab,'dtab',dtab,'Tend',Tend);
% --- senaryo B: dusuk hiz, vx vx_floor (=1) altina iner: fren -600, delta rampasi 0 -> 0.1
Tend = 6;
Ftab = [0 0; 0.5 0; 0.505 -600; Tend -600];
dtab = [0 0; 0.2 0; 0.25 0.1; Tend 0.1];
cases(2) = struct('name','B','v0',3,'Ftab',Ftab,'dtab',dtab,'Tend',Tend);
% --- senaryo C: buyuk delta, isaret degistiren ivmelenme/fren
Tend = 10;
Ftab = [0 1500; 3 1500; 3.002 -3000; 5 -3000; 5.002 1000; Tend 1000];
td = (0:0.005:Tend)'; dtab = [td 0.15*sin(1.3*td).*(td<8)];
cases(3) = struct('name','C','v0',15,'Ftab',Ftab,'dtab',dtab,'Tend',Tend);

names = {'X','Y','psi','vx','vy','r'};
err = zeros(numel(cases), 6);
tcmp = zeros(numel(cases), 1);
for c = 1:numel(cases)
    K = cases(c);
    in = Simulink.SimulationInput(mdl);
    in = in.setVariable('T_end', K.Tend);
    in = in.setVariable('vx0_init', K.v0);
    in = in.setVariable('Fx_in', K.Ftab);
    in = in.setVariable('delta_in', K.dtab);
    out = sim(in);
    t = out.state.Time;
    x = out.state.Data;
    Xr = ref_solve(P, K.Ftab, K.dtab, [0 0 0 K.v0 0 0], t);
    valid = Xr(:,4) > 0.05;   % B: v_x -> 0 sinirindan hemen once kesilir
    E = abs(x(valid,:) - Xr(valid,:));
    err(c,:) = max(E, [], 1) ./ max(1, max(abs(Xr(valid,:)), [], 1));
    tcmp(c) = t(find(valid, 1, 'last'));
end

fprintf('\ntest_plant_wiring: olcekli hata max|sim - ref| / max(1, max|ref|), esik %.0e\n', tol);
fprintf('%-4s %8s', 'case', 't_cmp');
fprintf(' %10s', names{:});
fprintf('\n');
for c = 1:numel(cases)
    fprintf('%-4s %8.3f', cases(c).name, tcmp(c));
    fprintf(' %10.3e', err(c,:));
    fprintf('\n');
end
if any(err(:) >= tol)
    error('test_plant_wiring: esik asildi (max olcekli hata %.3e >= %.0e).', max(err(:)), tol);
end
fprintf('GECTI (max olcekli hata %.3e)\n', max(err(:)));
end

%% ======================================================================
function restore(mdl, oldpath, oldcfg, cache)
if bdIsLoaded(mdl), close_system(mdl, 0); end
Simulink.fileGenControl('setConfig', 'config', oldcfg);
path(oldpath);
if exist(cache, 'dir'), rmdir(cache, 's'); end
end

%% ======================================================================
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
if Fx >= 0, Fxf = 0; Fxr = Fx; else, Fxf = P.bs*Fx; Fxr = (1-P.bs)*Fx; end
vh = max(vx, P.vxf);
af = d - atan((vy + P.lf*r)/vh);
ar = -atan((vy - P.lr*r)/vh);
Fyf = P.Cf*af; Fyr = P.Cr*ar;
Fres = 0.5*P.rho*P.CdA*vx^2 + P.Crr*P.m*P.g*cos(P.th) + P.m*P.g*sin(P.th);
dvx = (Fxf*cos(d) - Fyf*sin(d) + Fxr - Fres)/P.m + vy*r;
if vx <= 0 && dvx < 0, dvx = 0; end
dvy = (Fyf*cos(d) + Fxf*sin(d) + Fyr)/P.m - vx*r;
dr  = (P.lf*(Fyf*cos(d) + Fxf*sin(d)) - P.lr*Fyr)/P.Iz;
dx = [vx*cos(psi) - vy*sin(psi); vx*sin(psi) + vy*cos(psi); r; dvx; dvy; dr];
end

function u = lininterp_hold(tab, t)
if t <= tab(1,1), u = tab(1,2); elseif t >= tab(end,1), u = tab(end,2);
else, u = interp1(tab(:,1), tab(:,2), t, 'linear'); end
end
