function test_closed_loop()
%TEST_CLOSED_LOOP  S3 kapali dongu testi (Lateral C + Longitudinal C, MPC).
%   Kosular: Rectangle 3 tur, Circle 3 tur, Rectangle
%   baslangic sapmasi (e_y0 = 1 m, e_psi0 = 0.1 rad, 5.5 s, duzlukte).
%   a) tur Bitis ile biter; b) t > 5 s takip: |e_y| <= 0.3 m, |e_x| <= 1 m,
%   |e_psi - e_psi,ss| <= 0.1 rad (e_psi,ss = -kappa (lr - m v^2 lf/(Cr L)):
%   Rajamani 3.2 kalici yaw hatasi, geri beslemeyle sifirlanamaz; ham
%   max|e_psi| ayrica yazdirilir); c) oturma <= 3 s (|e_y| < 0.1), asma
%   <= %30, oturduktan sonra delta isaret degisimi <= 2 (|delta| > 1e-4);
%   d) lastik kullanimi sqrt(Fx^2 + Fy^2)/(mu Fz) <= 0.95, |a_y| <= mu g;
%   e) iki MPC'de exitflag her adimda 1; uyari yok.
%   Kullanim (proje kokunden):  addpath tests; test_closed_loop

root = fileparts(fileparts(mfilename('fullpath')));
mdl  = 'VehicleDiagram';
wasLoaded = bdIsLoaded(mdl);
oldpath = addpath(root);
cache   = tempname;
oldcfg  = Simulink.fileGenControl('getConfig');
Simulink.fileGenControl('set', 'CacheFolder', cache, 'CodeGenFolder', cache, 'createDir', true);
cleanup = onCleanup(@() restore(mdl, wasLoaded, oldpath, oldcfg, cache)); %#ok<NASGU>
res = {};  nw = 0;

for q = {{'Rectangle', 3}, {'Circle', 3}}
    [nm, nl] = q{1}{:};
    evalin('base', ['map_name = ''' nm '''; Model_Setup']);
    P = evalin('base', 'struct(''m'',m,''lf'',lf,''lr'',lr,''Cr'',Cr,''L'',L,''mu'',mu,''g'',g)');
    fin = evalin('base', 'fin');  fin.n_laps = nl;
    tic;  out = sim(Simulink.SimulationInput(mdl).setVariable('fin', fin));  tw = toc;
    nw = nw + numel(out.SimulationMetadata.ExecutionInfo.WarningDiagnostics);
    t = out.err.Time;  E = out.err.Data;  m = t > 5;
    vx = interp1(out.state.Time, out.state.Data(:,4), t);
    k  = out.kappa_p.Data;
    eps_ss = -k.*(P.lr - P.m*vx.^2*P.lf/(P.Cr*P.L));
    res(end+1,:) = chk(sprintf('a) %s %d tur: Bitis (done = 1, lap = n_laps) hata', nm, nl), ...
        (out.done.Data(end) ~= 1) + (out.lap.Data(end) ~= nl), 0.5); %#ok<AGROW>
    res(end+1,:) = chk(['b) ' nm ' max|e_y| (t > 5 s) [m]'], max(abs(E(m,2))), 0.3); %#ok<AGROW>
    res(end+1,:) = chk(['b) ' nm ' max|e_psi - e_psi,ss| [rad]'], max(abs(E(m,3) - eps_ss(m))), 0.1); %#ok<AGROW>
    res(end+1,:) = chk(['b) ' nm ' max|e_x| [m]'], max(abs(E(m,1))), 1); %#ok<AGROW>
    [uf, ur, ay] = tire_use(out, P);
    res(end+1,:) = chk(['d) ' nm ' max lastik kullanimi (on, arka)'], max([uf; ur]), 0.95); %#ok<AGROW>
    res(end+1,:) = chk(['d) ' nm ' max|a_y| / (mu g)'], max(abs(ay))/(P.mu*P.g), 1); %#ok<AGROW>
    res(end+1,:) = chk(['e) ' nm ' exitflag ~= 1 adim sayisi'], ...
        nnz(out.flag_lat.Data ~= 1) + nnz(out.flag_lon.Data ~= 1), 0.5); %#ok<AGROW>
    fprintf(['%s %d tur: sim %.1f s, duvar %.1f s; ham max|e_psi| %.4f rad (duzluk %.4f), ' ...
             'max|e_psi,ss| %.4f; max(v_x - v_ref_p) %.3f, min %.3f m/s; on %.3f arka %.3f\n'], ...
        nm, nl, t(end), tw, max(abs(E(m,3))), max([0; abs(E(m & k == 0, 3))]), max(abs(eps_ss)), ...
        max(vx(m) - out.v_ref_p.Data(m)), min(vx(m) - out.v_ref_p.Data(m)), max(uf), max(ur));
end

% c) baslangic sapmasi: yolun basinda e_y0 = 1 m (sola), e_psi0 = 0.1 rad
evalin('base', 'map_name = ''Rectangle''; Model_Setup');
ref = evalin('base', 'ref');  th = ref.theta0;
in = Simulink.SimulationInput(mdl);
in = in.setVariable('X0', ref.x(ref.idx0) - sin(th));
in = in.setVariable('Y0', ref.y(ref.idx0) + cos(th));
in = in.setVariable('psi0', th + 0.1);
in = in.setVariable('T_end', 5.5);   % egrilik ~6.4 s'de baslar; boyuna MPC onizlemeyle ~5.0 s'de frenler (v_ref 5.3 s'de duser)
in = in.setVariable('stop_enable', 0);
out = sim(in);
nw = nw + numel(out.SimulationMetadata.ExecutionInfo.WarningDiagnostics);
t = out.err.Time;  ey = out.err.Data(:,2);  d = out.delta_cmd.Data;
ks = find(abs(ey) >= 0.1, 1, 'last') + 1;   % bundan sonra |e_y| < 0.1
ds = d(ks:end);  ds = ds(abs(ds) > 1e-4);
res(end+1,:) = chk('c) oturma suresi |e_y| < 0.1 [s]', t(ks), 3);
res(end+1,:) = chk('c) asma (e_y0 = 1 m) [oran]', max(0, -min(ey)), 0.3);
res(end+1,:) = chk('c) oturduktan sonra delta isaret degisimi', nnz(diff(sign(ds)) ~= 0), 2.5);
fprintf('c) bilgi: e_y0 %.3f m, e_psi0 %.3f rad; oturma %.2f s, std(delta) oturduktan sonra %.2e rad, max|delta| %.3f rad\n', ...
    ey(1), out.err.Data(1,3), t(ks), std(d(ks:end)), max(abs(d)));
res(end+1,:) = chk('uyari sayisi (3 kosu)', nw, 0.5);

fprintf('\ntest_closed_loop:\n');
for i = 1:size(res,1)
    fprintf('%-58s %12.4e %10.2e %s\n', res{i,1}, res{i,2}, res{i,3}, tern(res{i,4}));
end
nf = nnz(~[res{:,4}]);
if nf > 0, error('test_closed_loop: %d olcut KALDI.', nf); end
fprintf('GECTI (%d olcut)\n', size(res,1));
end

%% ======================================================================
function [uf, ur, ay] = tire_use(out, P)
% aks basina lastik kullanimi ve yanal ivme (logsout, her 1 ms)
g = @(n) out.logsout.get(n).Values.Data;
uf = hypot(g('F_xf'), g('F_yf')) ./ (P.mu*g('F_zf'));
ur = hypot(g('F_xr'), g('F_yr')) ./ (P.mu*g('F_zr'));
ay = g('vy_dot') + out.state.Data(:,4).*out.state.Data(:,6);
end

function restore(mdl, wasLoaded, oldpath, oldcfg, cache)
if ~wasLoaded && bdIsLoaded(mdl), close_system(mdl, 0); end
Simulink.fileGenControl('setConfig', 'config', oldcfg);
path(oldpath);
if exist(cache, 'dir'), rmdir(cache, 's'); end
end

function row = chk(name, val, tol)
row = {name, double(val), tol, double(val) <= tol};
end

function s = tern(b)
if b, s = 'GECTI'; else, s = 'KALDI'; end
end
