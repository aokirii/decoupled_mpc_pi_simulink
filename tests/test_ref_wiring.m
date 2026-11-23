function test_ref_wiring()
%TEST_REF_WIRING  S2 Simulink entegrasyon testi (Referans, Hata Hesabi, Bitis).
%   (a) Kablolama: tam lastik modu, Rectangle baslangici, acik cevrim
%       (F_x = 300 N, delta = 0.03 sin 0.7t), 8 s, stop_enable = 0. Plant
%       durumu 0.02 s anlarinda logdan alinir; ref_advance, ref_error ve
%       ref_done MATLAB'da ayni geri besleme sirasiyla (Unit Delay: s_ref
%       IC s0, idx / lap / s_proj / n_slow IC 0) yeniden kosturulur ve
%       Simulink loglariyla karsilastirilir.
%   (b) Durma bitisi: F_x = -6000 N (t >= 0.5 s). stop_enable = 1 iken
%       ilk yavas ornekten t_hold sonra biter; stop_enable = 0 iken bitmez.
%   (c) Tur bitisi: fin.n_laps = 0.05, (a) girisleri; s_veh - s0 >=
%       n_laps L oldugu ilk adimda biter.
%   Ayrica: uyari yok, cebirsel dongu yok (AlgebraicLoopMsg = error),
%   ReleaseName R2024b, kontrol loglari Ts_ctrl araliginda.
%   Olcut asilirsa error. Kullanim (proje kokunden):
%       addpath tests; test_ref_wiring
%   Gereken: VehicleDiagram.slx guncel olmali (build_model).

tol  = 1e-9;
root = fileparts(fileparts(mfilename('fullpath')));
mdl  = 'VehicleDiagram';

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
ref = evalin('base', 'ref');   fin = evalin('base', 'fin');
Ts  = evalin('base', 'Ts_ctrl');
wf  = evalin('base', 'win_fwd');  wb = evalin('base', 'win_back');
res = {};

info = Simulink.MDLInfo(fullfile(root, [mdl '.slx']));
res(end+1,:) = chk('ReleaseName = R2024b', ~strcmp(info.ReleaseName, 'R2024b'), 0.5);
load_system(mdl);
res(end+1,:) = chk('AlgebraicLoopMsg = error', ~strcmp(get_param(mdl, 'AlgebraicLoopMsg'), 'error'), 0.5);

td = (0:0.01:8).';
Fa = [0 300; 8 300];  Da = [td 0.03*sin(0.7*td)];

%% ---------------- (a) kablolama ----------------
oa = simrun(mdl, 8, Fa, Da, 0, []);
nw = nwarn(oa);
tk = oa.err.Time;
res(end+1,:) = chk('(a) kontrol log araligi max|dt - Ts_ctrl|', max(abs(diff(tk) - Ts)), 1e-12);
res(end+1,:) = chk('(a) ilk log ani t_0', abs(tk(1)), 1e-12);
Xk = state_at(oa, tk);
M  = rerun(Xk, ref, fin, Ts, wf, wb, 0);
cmp = {'ref_pt', M.ref_pt; 'err', M.err; 's_veh', M.s_veh; 's_proj', M.s_proj; ...
       'kappa_p', M.kappa_p; 'v_ref_p', M.v_ref_p; 's_dot', M.s_dot};
for i = 1:size(cmp, 1)
    d = max(abs(oa.(cmp{i,1}).Data - cmp{i,2}), [], 'all');
    res(end+1,:) = chk(sprintf('(a) %s max|Simulink - MATLAB|', cmp{i,1}), d, tol); %#ok<AGROW>
end
res(end+1,:) = chk('(a) idx / lap / done farkli ornek sayisi', ...
    nnz(oa.idx.Data ~= M.idx) + nnz(oa.lap.Data ~= M.lap) + nnz(oa.done.Data ~= M.done), 0.5);
fprintf(['(a) bilgi: %d kontrol adimi; s_ref %.3f -> %.3f m, s_veh %.3f -> %.3f m; ' ...
         'e_y [%.3f, %.3f] m, e_x [%.3f, %.3f] m; lap %d -> %d\n'], numel(tk), ...
    M.ref_pt(1,1), M.ref_pt(end,1), M.s_veh(1), M.s_veh(end), min(M.err(:,2)), max(M.err(:,2)), ...
    min(M.err(:,1)), max(M.err(:,1)), M.lap(1), M.lap(end));

%% ---------------- (b) durma bitisi ----------------
Fb = [0 0; 0.5 0; 0.505 -6000; 6 -6000];  Db = [0 0; 6 0];
ob0 = simrun(mdl, 6, Fb, Db, 0, []);
ob1 = simrun(mdl, 6, Fb, Db, 1, []);
nw = nw + nwarn(ob0) + nwarn(ob1);
tb = ob0.err.Time;  Xb = state_at(ob0, tb);
nsl = 0;  kexp = 0;  k0 = 0;
for k = 1:numel(tb)            % beklenen adim, bagimsiz: (n_slow - 1) Ts >= t_hold
    if hypot(Xb(k,4), Xb(k,5)) < fin.v_stop, nsl = nsl + 1; else, nsl = 0; end
    if nsl == 1, k0 = k; end
    if nsl >= 1 && (nsl - 1)*Ts >= fin.t_hold - 1e-9, kexp = k; break; end
end
assert(kexp > 0, '(b) senaryoda arac durmadi');
res(end+1,:) = chk('(b) stop_enable = 1: bitis ani - beklenen [s]', abs(ob1.done.Time(end) - tb(kexp)), 1e-9);
res(end+1,:) = chk('(b) stop_enable = 1: son done = 1, oncekiler 0 (hata sayisi)', ...
    (ob1.done.Data(end) ~= 1) + nnz(ob1.done.Data(1:end-1)), 0.5);
res(end+1,:) = chk('(b) stop_enable = 1: state logu bitis aninda biter [s]', abs(ob1.state.Time(end) - tb(kexp)), 1e-9);
res(end+1,:) = chk('(b) stop_enable = 0: T_end''e kadar kosar [s]', abs(ob0.state.Time(end) - 6), 1e-9);
res(end+1,:) = chk('(b) stop_enable = 0: done = 1 sayisi', nnz(ob0.done.Data), 0.5);
fprintf('(b) bilgi: ilk yavas ornek t = %.2f s, bitis t = %.2f s (beklenen %.2f s)\n', ...
    tb(k0), ob1.done.Time(end), tb(kexp));

%% ---------------- (c) tur bitisi ----------------
fc = fin;  fc.n_laps = 0.05;
kc = find(M.s_veh - ref.s0 >= fc.n_laps*ref.L, 1);   % (a)'nin MATLAB yeniden kosusu
assert(~isempty(kc), '(c) 8 s icinde 0.05 tur tamamlanmadi');
oc = simrun(mdl, 8, Fa, Da, 1, fc);
nw = nw + nwarn(oc);
res(end+1,:) = chk('(c) bitis ani - beklenen [s]', abs(oc.done.Time(end) - tk(kc)), 1e-9);
res(end+1,:) = chk('(c) son done = 1, oncekiler 0 (hata sayisi)', ...
    (oc.done.Data(end) ~= 1) + nnz(oc.done.Data(1:end-1)), 0.5);
fprintf('(c) bilgi: 0.05 L = %.3f m; bitis t = %.2f s, s_veh - s0 = %.4f m (bir adim once %.4f m)\n', ...
    fc.n_laps*ref.L, oc.done.Time(end), M.s_veh(kc) - ref.s0, M.s_veh(kc-1) - ref.s0);

res(end+1,:) = chk('uyari sayisi (4 kosu)', nw, 0.5);

%% ---------------- Ozet ----------------
fprintf('\ntest_ref_wiring:\n');
fprintf('%-62s %12s %10s %s\n', 'olcut', 'deger', 'esik', 'sonuc');
for i = 1:size(res,1)
    fprintf('%-62s %12.4e %10.2e %s\n', res{i,1}, res{i,2}, res{i,3}, tern(res{i,4}));
end
nf = nnz(~[res{:,4}]);
if nf > 0
    error('test_ref_wiring: %d olcut KALDI.', nf);
end
fprintf('GECTI (%d olcut)\n', size(res,1));
end

%% ======================================================================
function out = simrun(mdl, Tend, Ftab, dtab, stop_en, fin)
in = Simulink.SimulationInput(mdl);
in = in.setVariable('T_end', Tend);
in = in.setVariable('Fx_in', Ftab);
in = in.setVariable('delta_in', dtab);
in = in.setVariable('stop_enable', stop_en);
if ~isempty(fin), in = in.setVariable('fin', fin); end
out = sim(in);
end

function n = nwarn(out)
n = numel(out.SimulationMetadata.ExecutionInfo.WarningDiagnostics);
end

function Xk = state_at(out, tk)
% surekli state logundan (her 1 ms) kontrol anlarindaki ornekler
t = out.state.Time;  Xk = zeros(numel(tk), 6);
for k = 1:numel(tk)
    i = find(abs(t - tk(k)) < 1e-9, 1);
    assert(~isempty(i), 'state logunda t = %g yok', tk(k));
    Xk(k,:) = out.state.Data(i,:);
end
end

function M = rerun(Xk, ref, fin, Ts, wf, wb, stop_en)
% Simulink'teki sira: k aninda Unit Delay cikislari (s_ref, idx, lap,
% s_proj, n_slow) onceki adimin degerleri; guncelleme adim sonunda.
n = size(Xk, 1);
M = struct('ref_pt', zeros(n,6), 'err', zeros(n,6), 's_veh', zeros(n,1), ...
    's_proj', zeros(n,1), 'idx', zeros(n,1), 'lap', zeros(n,1), ...
    'kappa_p', zeros(n,1), 'v_ref_p', zeros(n,1), 's_dot', zeros(n,1), 'done', zeros(n,1));
s = ref.s0;  idx = 0;  lap = 0;  sp = 0;  ns = 0;
N = numel(ref.theta);
for k = 1:n
    u = s/ref.ds;  j = floor(u);  t = u - j;  a = mod(j, N) + 1;  b = mod(j + 1, N) + 1;
    dth = mod(ref.theta(b) - ref.theta(a) + pi, 2*pi) - pi;
    M.ref_pt(k,:) = [s, periodic_interp(ref.x, s, ref.ds), periodic_interp(ref.y, s, ref.ds), ...
        mod(ref.theta(a) + t*dth + pi, 2*pi) - pi, periodic_interp(ref.v, s, ref.ds), ...
        periodic_interp(ref.kappa, s, ref.ds)];
    x = Xk(k,:);
    [ex, ey, ep, exd, eyd, epd, sv, sp, idx, lap, ~, vrp, kp, ~, sdt] = ...
        ref_error(x(1), x(2), x(3), x(4), x(5), x(6), s, idx, lap, sp, ref, wf, wb);
    M.err(k,:) = [ex ey ep exd eyd epd];
    M.s_veh(k) = sv;  M.s_proj(k) = sp;  M.idx(k) = idx;  M.lap(k) = lap;
    M.kappa_p(k) = kp;  M.v_ref_p(k) = vrp;  M.s_dot(k) = sdt;
    [M.done(k), ns] = ref_done(sv, x(4), x(5), ns, stop_en, ref, fin, Ts);
    s = ref_advance(s, ref, Ts);
end
end

function restore(mdl, wasLoaded, oldpath, oldcfg, cache)
if ~wasLoaded && bdIsLoaded(mdl), close_system(mdl, 0); end
Simulink.fileGenControl('setConfig', 'config', oldcfg);
path(oldpath);
if exist(cache, 'dir'), rmdir(cache, 's'); end
end

function row = chk(name, val, tol)
row = {name, double(val), tol, double(val) < tol};
end

function s = tern(b)
if b, s = 'GECTI'; else, s = 'KALDI'; end
end
