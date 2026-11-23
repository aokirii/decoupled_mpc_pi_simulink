function build_model()
%BUILD_MODEL  VehicleDiagram.slx'i sifirdan uretir (S1: Plant, acik cevrim).
%   Blok yapisi el cizimi diyagrami izler:
%     Plant = [Dinamik Denklem -> Integrator (hiz) -> Kinematik Denklem
%              -> Integrator (konum)], geri beslemeler v_x, v_y, r, psi.
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
% Yerlesim cizimdeki gibi: Dinamik sol ust, Integrator (hiz) sag ust,
% Kinematik sag alt, Integrator (konum) sol alt; alt sira sagdan sola akar.

add_block('built-in/Inport', [plant '/F_x'],   'Position', [30 63 60 77]);
add_block('built-in/Inport', [plant '/delta'], 'Port', '2', ...
    'Position', [30 103 60 117]);

% --- Dinamik Denklem (sol ust) ---
dyn = [plant '/Dinamik Denklem'];
add_block('simulink/User-Defined Functions/MATLAB Function', dyn, ...
    'Position', [140 40 340 240]);
set_mfunction(dyn, dyn_script(), ...
    {'m','Iz','lf','lr','Cf','Cr','rho_air','CdA','C_rr','g', ...
     'theta_road','vx_floor','brake_split'});

% --- Integrator (hiz) (sag ust) ---
vint = [plant '/Integrator (hiz)'];
add_block('built-in/Subsystem', vint, 'Position', [460 70 580 210]);
build_integrator(vint, {'vx_dot','vy_dot','r_dot'}, {'v_x','v_y','r'}, ...
    {'vx0_init','vy0_init','r0_init'}, [true false false]);

% --- Kinematik Denklem (sag alt, sola bakar) ---
kin = [plant '/Kinematik Denklem'];
add_block('simulink/User-Defined Functions/MATLAB Function', kin, ...
    'Position', [440 320 600 460], 'Orientation', 'left');
set_mfunction(kin, kin_script(), {});

% --- Integrator (konum) (sol alt, sola bakar) ---
pint = [plant '/Integrator (konum)'];
add_block('built-in/Subsystem', pint, 'Position', [160 330 280 450], ...
    'Orientation', 'left');
build_integrator(pint, {'X_dot','Y_dot','psi_dot'}, {'X','Y','psi'}, ...
    {'X0','Y0','psi0'}, [false false false]);

% --- Cikis: durum vektoru [X Y psi vx vy r] ---
% Turevler cizgi yerine sinyal loglamasiyla (logsout) kaydedilir.
add_block('built-in/Mux', [plant '/Mux_state'], 'Inputs', '6', ...
    'Position', [720 180 725 400]);
add_block('built-in/Outport', [plant '/state'], 'Position', [790 283 820 297]);

al = {'autorouting', 'smart'};
D = 'Dinamik Denklem'; V = 'Integrator (hiz)';
K = 'Kinematik Denklem'; P = 'Integrator (konum)';
% girisler
add_line(plant, 'F_x/1',   [D '/1'], al{:});
add_line(plant, 'delta/1', [D '/2'], al{:});
% Dinamik -> Integrator (hiz)
for i = 1:3, add_line(plant, sprintf('%s/%d', D, i), sprintf('%s/%d', V, i), al{:}); end
% Integrator (hiz) -> Dinamik (geri besleme) ve Kinematik
for i = 1:3
    add_line(plant, sprintf('%s/%d', V, i), sprintf('%s/%d', D, i + 2), al{:});
    add_line(plant, sprintf('%s/%d', V, i), sprintf('%s/%d', K, i), al{:});
end
% Kinematik -> Integrator (konum); psi -> Kinematik
for i = 1:3, add_line(plant, sprintf('%s/%d', K, i), sprintf('%s/%d', P, i), al{:}); end
add_line(plant, [P '/3'], [K '/4'], al{:});
% durum vektoru [X Y psi vx vy r]
for i = 1:3, add_line(plant, sprintf('%s/%d', P, i), sprintf('Mux_state/%d', i), al{:}); end
for i = 1:3, add_line(plant, sprintf('%s/%d', V, i), sprintf('Mux_state/%d', i + 3), al{:}); end
add_line(plant, 'Mux_state/1', 'state/1', al{:});

% sinyal adlari
names = {'vx_dot','vy_dot','r_dot'};
for i = 1:3, name_line([plant '/' D], i, names{i}); end
names = {'v_x','v_y','r'};
for i = 1:3, name_line([plant '/' V], i, names{i}); end
names = {'X_dot','Y_dot','psi_dot'};
for i = 1:3, name_line([plant '/' K], i, names{i}); end
names = {'X','Y','psi'};
for i = 1:3, name_line([plant '/' P], i, names{i}); end

% turevler logsout'a: vx_dot, vy_dot, r_dot, X_dot, Y_dot, psi_dot
for blk = {D, K}
    ph = get_param([plant '/' blk{1}], 'PortHandles');
    for i = 1:3, set_param(ph.Outport(i), 'DataLogging', 'on'); end
end
end

%% ======================================================================
function build_integrator(sub, in_names, out_names, ics, floor0)
% Uc ayri surekli Integrator; floor0(i) = true ise alt sinir 0 (v_x).
for i = 1:3
    y = 30 + (i - 1)*60;
    add_block('built-in/Inport', [sub '/' in_names{i}], 'Port', num2str(i), ...
        'Position', [40 y 70 y + 14]);
    ib = [sub '/int_' out_names{i}];
    add_block('built-in/Integrator', ib, 'InitialCondition', ics{i}, ...
        'Position', [140 y - 8 175 y + 22]);
    if floor0(i)
        set_param(ib, 'LimitOutput', 'on', 'LowerSaturationLimit', '0', ...
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
function s = dyn_script()
s = strjoin({
'function [vx_dot, vy_dot, r_dot] = fcn(F_x, delta, v_x, v_y, r, m, Iz, lf, lr, Cf, Cr, rho_air, CdA, C_rr, g, theta_road, vx_floor, brake_split)'
'% Dinamik Denklem: tam form bisiklet modeli, lineer lastik (Rajamani 2.3).'
'% Girisler: F_x [N], delta [rad], v_x, v_y [m/s], r = psi_dot [rad/s].'
''
'% F_x dagilimi: itis yalniz arkadan; fren brake_split ile on/arka.'
'if F_x >= 0'
'    Fxf = 0;'
'    Fxr = F_x;'
'else'
'    Fxf = brake_split*F_x;'
'    Fxr = (1 - brake_split)*F_x;'
'end'
''
'% Kayma acilari (bolmede v_x tabani).'
'vx_hat  = max(v_x, vx_floor);'
'alpha_f = delta - atan((v_y + lf*r)/vx_hat);'
'alpha_r = -atan((v_y - lr*r)/vx_hat);'
''
'% Lineer lastik.'
'Fyf = Cf*alpha_f;'
'Fyr = Cr*alpha_r;'
''
'% Direnc: hava + yuvarlanma + egim.'
'F_res = 0.5*rho_air*CdA*v_x^2 + C_rr*m*g*cos(theta_road) + m*g*sin(theta_road);'
''
'vx_dot = (Fxf*cos(delta) - Fyf*sin(delta) + Fxr - F_res)/m + v_y*r;'
'vy_dot = (Fyf*cos(delta) + Fxf*sin(delta) + Fyr)/m - v_x*r;'
'r_dot  = (lf*(Fyf*cos(delta) + Fxf*sin(delta)) - lr*Fyr)/Iz;'
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
