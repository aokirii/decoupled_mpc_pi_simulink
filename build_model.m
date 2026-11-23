function build_model()
%BUILD_MODEL  VehicleDiagram.slx'i sifirdan uretir (S1 + S1b: Plant, acik cevrim).
%   Blok yapisi el cizimi diyagrami izler:
%     Plant = [Dinamik Denklem -> Integrator (hiz) -> Kinematik Denklem
%              -> Integrator (konum)], geri beslemeler v_x, v_y, r, psi.
%   S1b tam tekerlek modeli: Aktuator -> Tekerlek Dinamigi -> Integrator
%   (tekerlek); Lastik Modeli (gecici kayma + Dugoff) -> Integrator (kayma);
%   Dinamik Denklem a_x -> Filtre a_x -> Yuk Transferi -> Lastik Modeli.
%   tire_mode = 0 ("notlar") S1 davranisini aynen verir; 1 = "tam".
%   Ust seviyede yalniz test girisleri (From Workspace: F_x, delta), Plant
%   ve loglama (To Workspace) vardir. Referans, Hata Hesabi, kontrolculer ve
%   Gorsellestirme S2-S4'te eklenir.
%   Parametreler base workspace'ten okunur: once Model_Setup calistirin.
%   Son dosya R2024b bicimine aktarilir (Mac uyumu).
%
%   UYARI: VehicleDiagram.slx'i R2025b'de elle acip KAYDETMEYIN. Kaydedilirse
%   dosya R2025b bicimine doner ve *.slx.r20* yedegi olusur. Modeli
%   degistirmek icin build_model.m'yi duzenleyip yeniden calistirin.
%   build_model acik VehicleDiagram'i kaydedilmemis degisiklikleriyle
%   birlikte sorgusuz kapatir (close_system(mdl, 0)).

mdl    = 'VehicleDiagram';
proj   = fileparts(mfilename('fullpath'));
target = fullfile(proj, [mdl '.slx']);

if bdIsLoaded(mdl), close_system(mdl, 0); end
% Yeniden uretimde eski VehicleDiagram.slx path'te oldugu icin cikan
% golgeleme uyarisi beklenen durumdur; yalniz new_system icin susturulur.
ws = warning('off', 'Simulink:Engine:MdlFileShadowing');
new_system(mdl);
warning(ws);

%% --- Model ayarlari ---
% ode4 sabit adim Ts_plant; kontrolculer (S3) Ts = 0.02 ile ayni modelde
% tek-gorevli (single-tasking) cok hizli calisir; 0.02 / 1e-3 tam sayidir.
set_param(mdl, ...
    'SolverType',             'Fixed-step', ...
    'Solver',                 'ode4', ...
    'FixedStep',              'Ts_plant', ...
    'StartTime',              '0', ...
    'StopTime',               'T_end', ...
    'EnableMultiTasking',     'off', ...
    'SampleTimeConstraint',   'unconstrained', ...
    'AlgebraicLoopMsg',       'error', ...
    'ArtificialAlgebraicLoopMsg', 'error', ...
    'ReturnWorkspaceOutputs', 'on', ...
    'SaveTime',               'off', ...
    'SaveOutput',             'off', ...
    'SignalLogging',          'on', ...
    'SignalLoggingName',      'logsout');

%% --- Ust seviye: test girisleri, Plant, loglama ---
fw = {'Interpolate', 'on', 'OutputAfterFinalValue', 'Holding final value', ...
      'SampleTime', '0'};
add_block('simulink/Sources/From Workspace', [mdl '/F_x'], ...
    'VariableName', 'Fx_in', fw{:}, 'Position', [40 63 120 87]);
add_block('simulink/Sources/From Workspace', [mdl '/delta'], ...
    'VariableName', 'delta_in', fw{:}, 'Position', [40 133 120 157]);

plant = [mdl '/Plant'];
add_block('built-in/Subsystem', plant, 'Position', [260 50 400 170]);

tw = {'SaveFormat', 'Timeseries', 'SampleTime', '-1', 'MaxDataPoints', 'inf'};
add_block('simulink/Sinks/To Workspace', [mdl '/log_state'], ...
    'VariableName', 'state', tw{:}, 'Position', [520 63 600 87]);
add_block('built-in/Mux', [mdl '/Mux_u'], 'Inputs', '2', ...
    'Position', [260 230 265 290]);
add_block('simulink/Sinks/To Workspace', [mdl '/log_input'], ...
    'VariableName', 'u_in', tw{:}, 'Position', [520 248 600 272]);

build_plant(plant);

al = {'autorouting', 'smart'};
add_line(mdl, 'F_x/1',   'Plant/1', al{:});
add_line(mdl, 'delta/1', 'Plant/2', al{:});
add_line(mdl, 'F_x/1',   'Mux_u/1', al{:});
add_line(mdl, 'delta/1', 'Mux_u/2', al{:});
add_line(mdl, 'Plant/1', 'log_state/1', al{:});
add_line(mdl, 'Mux_u/1', 'log_input/1', al{:});
name_line([mdl '/F_x'],   1, 'F_x');
name_line([mdl '/delta'], 1, 'delta');
name_line(plant, 1, 'state');
name_line([mdl '/Mux_u'], 1, 'u_in');

%% --- Kaydet: once yerel surum (gecici), sonra R2024b'ye aktar ---
% Disa aktarim gecici klasore yapilir; yalniz basarili olursa hedefin
% ustune tasinir. Hata olursa eski VehicleDiagram.slx yerinde kalir.
tmpd = tempname;
mkdir(fullfile(tmpd, 'export'));
cleanup = onCleanup(@() rmdir(tmpd, 's'));
save_system(mdl, fullfile(tmpd, [mdl '.slx']));
exported = fullfile(tmpd, 'export', [mdl '.slx']);
Simulink.exportToVersion(mdl, exported, 'R2024b');
close_system(mdl, 0);
movefile(exported, target, 'f');

info = Simulink.MDLInfo(target);
fprintf('build_model: %s yazildi (ReleaseName = %s).\n', target, info.ReleaseName);
end

%% ======================================================================
function build_plant(plant)
% Yerlesim: ust sira tekerlek zinciri (Aktuator -> Tekerlek Dinamigi ->
% Integrator (tekerlek)); sol orta Lastik Modeli + Integrator (kayma) +
% yuk transferi; sag tarafta S1 cekirdegi (Dinamik sol ust, Integrator
% (hiz) sag ust, Kinematik sag alt, Integrator (konum) sol alt).

add_block('built-in/Inport', [plant '/F_x'],   'Position', [30 43 60 57]);
add_block('built-in/Inport', [plant '/delta'], 'Port', '2', ...
    'Position', [30 323 60 337]);

% --- Aktuator: F_x komutu -> tekerlek torklari ---
act = [plant '/Aktuator'];
add_block('simulink/User-Defined Functions/MATLAB Function', act, ...
    'Position', [140 20 260 100]);
set_mfunction(act, act_script(), {'R_e', 'brake_split'});

% --- Tekerlek Dinamigi ---
whl = [plant '/Tekerlek Dinamigi'];
add_block('simulink/User-Defined Functions/MATLAB Function', whl, ...
    'Position', [400 20 540 140]);
set_mfunction(whl, whl_script(), {'tire_mode', 'I_w', 'R_e'});

% --- Integrator (tekerlek): omega >= 0 ---
wint = [plant '/Integrator (tekerlek)'];
add_block('built-in/Subsystem', wint, 'Position', [620 30 720 130]);
build_integrator(wint, {'omega_dot_f','omega_dot_r'}, {'omega_f','omega_r'}, ...
    {'vx0_init/R_e','vx0_init/R_e'}, {'0','0'});

% --- Lastik Modeli ---
tyr = [plant '/Lastik Modeli'];
add_block('simulink/User-Defined Functions/MATLAB Function', tyr, ...
    'Position', [140 200 340 560]);
set_mfunction(tyr, tyr_script(), ...
    {'tire_mode','lf','lr','Cf','Cr','C_kappa','mu','R_e','sigma_x', ...
     'sigma_y','k_low0','v_low','tau_c','vx_floor','brake_split'});

% --- Integrator (kayma): kappa >= -1 ---
sint = [plant '/Integrator (kayma)'];
add_block('built-in/Subsystem', sint, 'Position', [160 640 260 780], ...
    'Orientation', 'left');
build_integrator(sint, {'kappa_dot_f','kappa_dot_r','alpha_dot_f','alpha_dot_r'}, ...
    {'kappa_f','kappa_r','alpha_f','alpha_r'}, {'0','0','0','0'}, {'-1','-1','',''});
set_param(sint, 'ContentPreviewEnabled', 'off');

% --- a_x filtresi ve Yuk Transferi ---
add_block('simulink/Continuous/Transfer Fcn', [plant '/Filtre a_x'], ...
    'Numerator', '1', 'Denominator', '[tau_ax 1]', ...
    'Position', [440 830 520 870], 'Orientation', 'left');
lt = [plant '/Yuk Transferi'];
add_block('simulink/User-Defined Functions/MATLAB Function', lt, ...
    'Position', [160 820 280 880], 'Orientation', 'left');
set_mfunction(lt, lt_script(), {'m','g','lf','lr','L','h'});

% --- Dinamik Denklem (S1 cekirdegi sol ust) ---
dyn = [plant '/Dinamik Denklem'];
add_block('simulink/User-Defined Functions/MATLAB Function', dyn, ...
    'Position', [480 220 680 460]);
set_mfunction(dyn, dyn_script(), ...
    {'m','Iz','lf','lr','rho_air','CdA','C_rr','g','theta_road', ...
     'tire_mode','v_eps'});

% --- Integrator (hiz) (sag ust) ---
vint = [plant '/Integrator (hiz)'];
add_block('built-in/Subsystem', vint, 'Position', [800 250 920 390]);
build_integrator(vint, {'vx_dot','vy_dot','r_dot'}, {'v_x','v_y','r'}, ...
    {'vx0_init','vy0_init','r0_init'}, {'0 - 1e300*(tire_mode ~= 0)','',''});
% v_x alt siniri: notlar modunda 0 (S1), tam modda yok (-1e300). Ifade
% notlar modunda tam +0 verir (-inf*0 = NaN tuzagi yok).

% --- Kinematik Denklem (sag alt, sola bakar) ---
kin = [plant '/Kinematik Denklem'];
add_block('simulink/User-Defined Functions/MATLAB Function', kin, ...
    'Position', [780 560 940 700], 'Orientation', 'left');
set_mfunction(kin, kin_script(), {});

% --- Integrator (konum) (sol alt, sola bakar) ---
pint = [plant '/Integrator (konum)'];
add_block('built-in/Subsystem', pint, 'Position', [500 570 620 690], ...
    'Orientation', 'left');
build_integrator(pint, {'X_dot','Y_dot','psi_dot'}, {'X','Y','psi'}, ...
    {'X0','Y0','psi0'}, {'','',''});
set_param(pint, 'ContentPreviewEnabled', 'off');

% --- Cikis: durum vektoru [X Y psi vx vy r] ---
% Yeni buyuklukler ve turevler cizgi yerine sinyal loglamasiyla (logsout).
add_block('built-in/Mux', [plant '/Mux_state'], 'Inputs', '6', ...
    'Position', [1060 360 1065 580]);
add_block('built-in/Outport', [plant '/state'], 'Position', [1130 463 1160 477]);

al = {'autorouting', 'smart'};
A = 'Aktuator'; W = 'Tekerlek Dinamigi'; WI = 'Integrator (tekerlek)';
T = 'Lastik Modeli'; SI = 'Integrator (kayma)';
F = 'Filtre a_x'; LT = 'Yuk Transferi';
D = 'Dinamik Denklem'; V = 'Integrator (hiz)';
K = 'Kinematik Denklem'; P = 'Integrator (konum)';
ln = @(src, sp, dst, dp) add_line(plant, sprintf('%s/%d', src, sp), ...
    sprintf('%s/%d', dst, dp), al{:});

% girisler
ln('F_x', 1, A, 1);
ln('F_x', 1, T, 1);
ln('delta', 1, T, 2);
ln('delta', 1, D, 5);
% Aktuator -> Tekerlek Dinamigi -> Integrator (tekerlek) -> Lastik Modeli
ln(A, 1, W, 1); ln(A, 2, W, 2);
ln(W, 1, WI, 1); ln(W, 2, WI, 2);
ln(WI, 1, T, 6); ln(WI, 2, T, 7);
% Lastik Modeli -> Dinamik Denklem [F_xf F_yf F_xr F_yr], Tekerlek Dinamigi [F_xf F_xr]
for i = 1:4, ln(T, i, D, i); end
ln(T, 1, W, 3); ln(T, 3, W, 4);
% Lastik Modeli <-> Integrator (kayma)
for i = 1:4, ln(T, i + 4, SI, i); ln(SI, i, T, i + 7); end
% Dinamik a_x -> Filtre -> Yuk Transferi -> Lastik Modeli
ln(D, 4, F, 1); ln(F, 1, LT, 1);
ln(LT, 1, T, 12); ln(LT, 2, T, 13);
% S1 cekirdegi
for i = 1:3, ln(D, i, V, i); end
for i = 1:3
    ln(V, i, D, i + 5);
    ln(V, i, K, i);
    ln(V, i, T, i + 2);
end
for i = 1:3, ln(K, i, P, i); end
ln(P, 3, K, 4);
for i = 1:3, ln(P, i, 'Mux_state', i); end
for i = 1:3, ln(V, i, 'Mux_state', i + 3); end
ln('Mux_state', 1, 'state', 1);

% sinyal adlari; logla = true olanlar logsout'a
sig = {
    D,  {'vx_dot','vy_dot','r_dot','a_x'},               true
    V,  {'v_x','v_y','r'},                                false
    K,  {'X_dot','Y_dot','psi_dot'},                      true
    P,  {'X','Y','psi'},                                  false
    A,  {'T_f','T_r'},                                    true
    W,  {'omega_dot_f','omega_dot_r'},                    false
    WI, {'omega_f','omega_r'},                            true
    T,  {'F_xf','F_yf','F_xr','F_yr', ...
         'kappa_dot_f','kappa_dot_r','alpha_dot_f','alpha_dot_r'}, true
    SI, {'kappa_f','kappa_r','alpha_f','alpha_r'},        true
    F,  {'a_x_f'},                                        true
    LT, {'F_zf','F_zr'},                                  true
    };
for k = 1:size(sig, 1)
    ph = get_param([plant '/' sig{k, 1}], 'PortHandles');
    for i = 1:numel(sig{k, 2})
        set_param(get_param(ph.Outport(i), 'Line'), 'Name', sig{k, 2}{i});
        if sig{k, 3} && ~startsWith(sig{k, 2}{i}, {'kappa_dot', 'alpha_dot'})
            set_param(ph.Outport(i), 'DataLogging', 'on');
        end
    end
end
end

%% ======================================================================
function build_integrator(sub, in_names, out_names, ics, lower)
% Ayri surekli Integrator'lar; lower{i} bos degilse alt sinir (v_x >= 0,
% omega >= 0, kappa >= -1).
for i = 1:numel(in_names)
    y = 30 + (i - 1)*60;
    add_block('built-in/Inport', [sub '/' in_names{i}], 'Port', num2str(i), ...
        'Position', [40 y 70 y + 14]);
    ib = [sub '/int_' out_names{i}];
    add_block('built-in/Integrator', ib, 'InitialCondition', ics{i}, ...
        'Position', [140 y - 8 175 y + 22]);
    if ~isempty(lower{i})
        set_param(ib, 'LimitOutput', 'on', 'LowerSaturationLimit', lower{i}, ...
            'UpperSaturationLimit', 'inf');
    end
    add_block('built-in/Outport', [sub '/' out_names{i}], 'Port', num2str(i), ...
        'Position', [250 y 280 y + 14]);
    add_line(sub, [in_names{i} '/1'], ['int_' out_names{i} '/1']);
    add_line(sub, ['int_' out_names{i} '/1'], [out_names{i} '/1']);
end
end

%% ======================================================================
function set_mfunction(blk, script, params)
% MATLAB Function govdesini yazar; params listesindekiler base workspace'ten
% gelen skaler 'Parameter' verisi olur (tunable struct kullanilmaz).
ch = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', blk);
ch.Script = script;
for k = 1:numel(params)
    d = ch.find('-isa', 'Stateflow.Data', 'Name', params{k});
    d.Scope = 'Parameter';
end
end

%% ======================================================================
function name_line(blk, port, name)
ph = get_param(blk, 'PortHandles');
set_param(get_param(ph.Outport(port), 'Line'), 'Name', name);
end

%% ======================================================================
function s = act_script()
s = strjoin({
'function [T_f, T_r] = fcn(F_x, R_e, brake_split)'
'% Aktuator: F_x komutu [N] -> aks torklari [N m]. Itis yalniz arkadan;'
'% fren brake_split ile on/arka (fren torku negatif, donuse karsi koyar).'
'if F_x >= 0'
'    T_f = 0;'
'    T_r = F_x*R_e;'
'else'
'    T_f = brake_split*F_x*R_e;'
'    T_r = (1 - brake_split)*F_x*R_e;'
'end'
}, newline);
end

function s = whl_script()
s = strjoin({
'function [omega_dot_f, omega_dot_r] = fcn(T_f, T_r, F_xf, F_xr, tire_mode, I_w, R_e)'
'% Tekerlek Dinamigi: I_w*omega_dot = T - R_e*F_x,lastik (aks basina).'
'% Notlar modunda (tire_mode = 0) omega sabit kalir (kullanilmaz).'
'if tire_mode == 0'
'    omega_dot_f = 0;'
'    omega_dot_r = 0;'
'else'
'    omega_dot_f = (T_f - R_e*F_xf)/I_w;'
'    omega_dot_r = (T_r - R_e*F_xr)/I_w;'
'end'
}, newline);
end

function s = tyr_script()
s = strjoin({
'function [F_xf, F_yf, F_xr, F_yr, kappa_dot_f, kappa_dot_r, alpha_dot_f, alpha_dot_r] = fcn(F_x, delta, v_x, v_y, r, omega_f, omega_r, kappa_f, kappa_r, alpha_f, alpha_r, F_zf, F_zr, tire_mode, lf, lr, Cf, Cr, C_kappa, mu, R_e, sigma_x, sigma_y, k_low0, v_low, tau_c, vx_floor, brake_split)'
'% Lastik Modeli. Kuvvetler teker cercevesinde, aks basina (iki teker toplu).'
'% tire_mode = 0 (notlar, S1): F_x dogrudan dagitilir, lineer lastik, anlik'
'%                atan kayma; kayma durumlari sabit kalir (turev 0).'
'% tire_mode = 1 (tam): gecici kayma (gevseme uzunlugu) + dusuk hiz sonumu'
'%                + dusuk hizda doyma cekisi + yon duzeltmeli Dugoff.'
'if tire_mode == 0'
'    if F_x >= 0'
'        F_xf = 0;'
'        F_xr = F_x;'
'    else'
'        F_xf = brake_split*F_x;'
'        F_xr = (1 - brake_split)*F_x;'
'    end'
'    vx_hat = max(v_x, vx_floor);'
'    a_f = delta - atan((v_y + lf*r)/vx_hat);'
'    a_r = -atan((v_y - lr*r)/vx_hat);'
'    F_yf = Cf*a_f;'
'    F_yr = Cr*a_r;'
'    kappa_dot_f = 0;'
'    kappa_dot_r = 0;'
'    alpha_dot_f = 0;'
'    alpha_dot_r = 0;'
'else'
'    % teker cercevesi hizlari'
'    vwx_f =  v_x*cos(delta) + (v_y + lf*r)*sin(delta);'
'    vwy_f = -v_x*sin(delta) + (v_y + lf*r)*cos(delta);'
'    vwx_r =  v_x;'
'    vwy_r =  v_y - lr*r;'
'    % gecici kayma: sigma*x_dot + |v_wx|*x = kayma hizi'
'    kappa_dot_f = (R_e*omega_f - vwx_f - abs(vwx_f)*kappa_f)/sigma_x;'
'    kappa_dot_r = (R_e*omega_r - vwx_r - abs(vwx_r)*kappa_r)/sigma_x;'
'    alpha_dot_f = (-vwy_f - abs(vwx_f)*alpha_f)/sigma_y;'
'    alpha_dot_r = (-vwy_r - abs(vwx_r)*alpha_r)/sigma_y;'
'    % dusuk hiz agirligi w in [0,1]: |v_wx| < v_low bolgesinde etkin'
'    wf = w_lowspeed(vwx_f, v_low);'
'    wr = w_lowspeed(vwx_r, v_low);'
'    % doyma cekisi: dusuk hizda kayma durumu lastik deformasyonudur'
'    % (u = sigma*kappa); deformasyon kayma esigini (mu*F_z/C) asamaz.'
'    kappa_dot_f = kappa_dot_f - wf*sat_excess(kappa_f, mu*F_zf/C_kappa)/tau_c;'
'    kappa_dot_r = kappa_dot_r - wr*sat_excess(kappa_r, mu*F_zr/C_kappa)/tau_c;'
'    alpha_dot_f = alpha_dot_f - wf*sat_excess(alpha_f, mu*F_zf/Cf)/tau_c;'
'    alpha_dot_r = alpha_dot_r - wr*sat_excess(alpha_r, mu*F_zr/Cr)/tau_c;'
'    % dusuk hiz sonumu (Pacejka k_Vlow formu) ile efektif kayma'
'    kf = k_low0*wf;'
'    kr = k_low0*wr;'
'    ke_f = max(kappa_f + kf/C_kappa*(R_e*omega_f - vwx_f), -1);'
'    ke_r = max(kappa_r + kr/C_kappa*(R_e*omega_r - vwx_r), -1);'
'    te_f = alpha_f - kf/Cf*vwy_f;'
'    te_r = alpha_r - kr/Cr*vwy_r;'
'    [F_xf, F_yf] = dugoff(ke_f, te_f, F_zf, C_kappa, Cf, mu);'
'    [F_xr, F_yr] = dugoff(ke_r, te_r, F_zr, C_kappa, Cr, mu);'
'end'
'end'
''
'function w = w_lowspeed(vwx, v_low)'
'% Dusuk hiz agirligi: |v_wx| = 0 iken 1, |v_wx| >= v_low iken 0 (kosinus).'
'V = abs(vwx);'
'if V < v_low'
'    w = 0.5*(1 + cos(pi*V/v_low));'
'else'
'    w = 0;'
'end'
'end'
''
'function e = sat_excess(x, xs)'
'% x''in [-xs, xs] disinda kalan kismi: x - clamp(x, -xs, xs).'
'e = x - min(max(x, -xs), xs);'
'end'
''
'function [Fx, Fy] = dugoff(kap, t, Fz, Cx, Cy, mu)'
'% Yon duzeltmeli Dugoff, t = tan(alpha). lambda >= 1: Rajamani 13. bolum'
'% lineer bolge. lambda < 1: buyukluk Dugoff (|F| = mu*F_z*(2-lambda)/2,'
'% (1+kappa) sadelesmis, kappa = -1 icin 0/0 yok); yon lambda agirligiyla'
'% Dugoff yonunden (Cx*kap, Cy*t)/S kayma hizinin tersine (kap, t)/n karisir.'
'u = 1 + kap;'
'S = sqrt((Cx*kap)^2 + (Cy*t)^2);'
'n = sqrt(kap^2 + t^2);'
'if 2*S <= mu*Fz*u'
'    Fx = Cx*kap/u;'
'    Fy = Cy*t/u;'
'elseif n == 0'
'    Fx = 0;   % erisilemez (S > 0 => n > 0); guvenli dal'
'    Fy = 0;'
'else'
'    lam = mu*Fz*u/(2*S);'
'    Fm  = mu*Fz*(2 - lam)/2;'
'    ex  = lam*Cx*kap/S + (1 - lam)*kap/n;'
'    ey  = lam*Cy*t/S   + (1 - lam)*t/n;'
'    e   = sqrt(ex^2 + ey^2);'
'    Fx  = Fm*ex/e;'
'    Fy  = Fm*ey/e;'
'end'
'end'
}, newline);
end

function s = lt_script()
s = strjoin({
'function [F_zf, F_zr] = fcn(a_x_f, m, g, lf, lr, L, h)'
'% Yuk Transferi: filtrelenmis a_x ile aks dusey yukleri (>= 0).'
'F_zf = max((m*g*lr - m*a_x_f*h)/L, 0);'
'F_zr = max((m*g*lf + m*a_x_f*h)/L, 0);'
}, newline);
end

function s = dyn_script()
s = strjoin({
'function [vx_dot, vy_dot, r_dot, a_x] = fcn(F_xf, F_yf, F_xr, F_yr, delta, v_x, v_y, r, m, Iz, lf, lr, rho_air, CdA, C_rr, g, theta_road, tire_mode, v_eps)'
'% Dinamik Denklem: tam form govde denklemleri (Rajamani 2.3), lastik'
'% kuvvetleri (teker cercevesi, aks basina) Lastik Modeli''nden gelir.'
''
'% Direnc: hava + yuvarlanma + egim. Notlar modu S1 ifadesi (v_x >= 0);'
'% tam modda v_x sinirsiz oldugundan yone duyarli (durgunda F_res = egim).'
'if tire_mode == 0'
'    F_res = 0.5*rho_air*CdA*v_x^2 + C_rr*m*g*cos(theta_road) + m*g*sin(theta_road);'
'else'
'    F_res = 0.5*rho_air*CdA*v_x*abs(v_x) + C_rr*m*g*cos(theta_road)*tanh(v_x/v_eps) + m*g*sin(theta_road);'
'end'
''
'vx_dot = (F_xf*cos(delta) - F_yf*sin(delta) + F_xr - F_res)/m + v_y*r;'
'vy_dot = (F_yf*cos(delta) + F_xf*sin(delta) + F_yr)/m - v_x*r;'
'r_dot  = (lf*(F_yf*cos(delta) + F_xf*sin(delta)) - lr*F_yr)/Iz;'
'a_x    = vx_dot - v_y*r;   % govde boyuna ivmesi (yuk transferi icin)'
}, newline);
end

function s = kin_script()
s = strjoin({
'function [X_dot, Y_dot, psi_dot] = fcn(v_x, v_y, r, psi)'
'% Kinematik Denklem: govde hizlarindan global konum turevleri.'
'X_dot   = v_x*cos(psi) - v_y*sin(psi);'
'Y_dot   = v_x*sin(psi) + v_y*cos(psi);'
'psi_dot = r;'
}, newline);
end
