function build_model()
%BUILD_MODEL  VehicleDiagram.slx'i sifirdan uretir (S1: Plant, sade lastik; S2:
%   Referans, Hata Hesabi, Bitis; S3: Lateral C, Longitudinal C, kapali dongu).
%   Blok yapisi el cizimi diyagrami izler, Plant soldan saga:
%     Yuk Transferi -> Lastik Modeli (Dugoff + surtunme cemberi) -> Dinamik
%     Denklem -> Integrator (hiz) -> Kinematik Denklem -> Integrator (konum),
%     geri beslemeler v_x, v_y, r, psi ve a_x (Memory a_x, bir adim).
%   Ust seviye: test girisleri (From Workspace: F_x, delta), Plant (acik
%   cevrim, surekli, Ts_plant), S2 bloklari ve loglama (To Workspace).
%   S2 (ayrik, Ts_ctrl; plant durumu ZOH ile orneklenir):
%     Referans    : Unit Delay s_ref (IC ref.s0) + ref_advance; cikislar
%                   s_ref, X_ref, Y_ref, psi_ref, v_ref, kappa_ref
%     Hata Hesabi : ref_error; geri besleme Unit Delay idx, lap, s_proj (IC 0)
%     Bitis       : ref_done + Unit Delay n_slow (IC 0) + Stop Simulation
%   S3 (ayrik, Ts_ctrl): Lateral C (lat_mpc -> delta), Longitudinal C
%     (lon_mpc -> F_x); ikisi de mpc_qp. ctrl_enable Switch'i: 1 -> MPC
%     komutlari, 0 -> test girisleri (Fx_in, delta_in; plant testleri).
%   Gorsellestirme (S4) sonra eklenir.
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
build_s2(mdl, tw);
build_s3(mdl, tw);

al = {'autorouting', 'smart'};
add_line(mdl, 'Sw_Fx/1',    'Plant/1', al{:});
add_line(mdl, 'Sw_delta/1', 'Plant/2', al{:});
add_line(mdl, 'Sw_Fx/1',    'Mux_u/1', al{:});
add_line(mdl, 'Sw_delta/1', 'Mux_u/2', al{:});
add_line(mdl, 'Plant/1', 'log_state/1', al{:});
add_line(mdl, 'Mux_u/1', 'log_input/1', al{:});
name_line([mdl '/Sw_Fx'],    1, 'F_x');
name_line([mdl '/Sw_delta'], 1, 'delta');
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
% Soldan saga tek sira: Yuk Transferi -> Lastik Modeli -> Dinamik Denklem
% -> Integrator (hiz) -> Kinematik Denklem -> Integrator (konum) -> Mux.
% Geri beslemeler: v_x, v_y, r (Lastik, Dinamik, Kinematik), psi (Kinematik),
% a_x (Dinamik -> Memory a_x -> Yuk Transferi; bir adim, cebirsel dongu yok).
mfb = 'simulink/User-Defined Functions/MATLAB Function';

add_block('built-in/Inport', [plant '/F_x'],   'Position', [30 93 60 107]);
add_block('built-in/Inport', [plant '/delta'], 'Port', '2', ...
    'Position', [30 223 60 237]);

% --- Yuk Transferi: onceki adimin a_x'inden aks dusey yukleri ---
lt = [plant '/Yuk Transferi'];
add_block(mfb, lt, 'Position', [120 70 240 130]);
set_mfunction(lt, lt_script(), {'m','g','lf','lr','L','h'});
add_block('built-in/Memory', [plant '/Memory a_x'], 'InitialCondition', '0', ...
    'Position', [400 370 440 400], 'Orientation', 'left');

% --- Lastik Modeli: dagitim, kirpma, Dugoff ---
tyr = [plant '/Lastik Modeli'];
add_block(mfb, tyr, 'Position', [320 60 480 320]);
set_mfunction(tyr, tyr_script(), ...
    {'lf','lr','Cf','Cr','mu','vx_floor','brake_split'});

% --- Dinamik Denklem ---
dyn = [plant '/Dinamik Denklem'];
add_block(mfb, dyn, 'Position', [560 60 720 320]);
set_mfunction(dyn, dyn_script(), ...
    {'m','Iz','lf','lr','rho_air','CdA','C_rr','g','theta_road'});

% --- Integrator (hiz): v_x >= 0 ---
vint = [plant '/Integrator (hiz)'];
add_block('built-in/Subsystem', vint, 'Position', [800 110 900 260]);
build_integrator(vint, {'vx_dot','vy_dot','r_dot'}, {'v_x','v_y','r'}, ...
    {'vx0_init','vy0_init','r0_init'}, {'0','',''});

% --- Kinematik Denklem ---
kin = [plant '/Kinematik Denklem'];
add_block(mfb, kin, 'Position', [980 110 1120 270]);
set_mfunction(kin, kin_script(), {});

% --- Integrator (konum) ---
pint = [plant '/Integrator (konum)'];
add_block('built-in/Subsystem', pint, 'Position', [1200 110 1300 260]);
build_integrator(pint, {'X_dot','Y_dot','psi_dot'}, {'X','Y','psi'}, ...
    {'X0','Y0','psi0'}, {'','',''});

% --- Cikis: durum vektoru [X Y psi vx vy r] ---
add_block('built-in/Mux', [plant '/Mux_state'], 'Inputs', '6', ...
    'Position', [1400 80 1405 380]);
add_block('built-in/Outport', [plant '/state'], 'Position', [1460 223 1490 237]);

al = {'autorouting', 'smart'};
LT = 'Yuk Transferi'; T = 'Lastik Modeli'; D = 'Dinamik Denklem';
V = 'Integrator (hiz)'; K = 'Kinematik Denklem'; P = 'Integrator (konum)';
ln = @(src, sp, dst, dp) add_line(plant, sprintf('%s/%d', src, sp), ...
    sprintf('%s/%d', dst, dp), al{:});

ln(D, 4, 'Memory a_x', 1);
ln('Memory a_x', 1, LT, 1);
ln('F_x', 1, T, 1);
ln('delta', 1, T, 2);
ln('delta', 1, D, 5);
ln(LT, 1, T, 6); ln(LT, 2, T, 7);
for i = 1:4, ln(T, i, D, i); end
for i = 1:3
    ln(D, i, V, i);
    ln(V, i, T, i + 2);
    ln(V, i, D, i + 5);
    ln(V, i, K, i);
    ln(K, i, P, i);
    ln(P, i, 'Mux_state', i);
    ln(V, i, 'Mux_state', i + 3);
end
ln(P, 3, K, 4);
ln('Mux_state', 1, 'state', 1);

% sinyal adlari; logla = true olanlar logsout'a
sig = {
    LT, {'F_zf','F_zr'},                   true
    T,  {'F_xf','F_yf','F_xr','F_yr'},     true
    D,  {'vx_dot','vy_dot','r_dot','a_x'}, true
    V,  {'v_x','v_y','r'},                 false
    K,  {'X_dot','Y_dot','psi_dot'},       true
    P,  {'X','Y','psi'},                   false
    };
for k = 1:size(sig, 1)
    ph = get_param([plant '/' sig{k, 1}], 'PortHandles');
    for i = 1:numel(sig{k, 2})
        set_param(get_param(ph.Outport(i), 'Line'), 'Name', sig{k, 2}{i});
        if sig{k, 3}
            set_param(ph.Outport(i), 'DataLogging', 'on');
        end
    end
end
end

%% ======================================================================
function build_s2(mdl, tw)
% S2 bloklari. Hepsi ayrik (Ts_ctrl); plant durumu ZOH ile orneklenir.
% Geri beslemeler Unit Delay: s_ref (IC ref.s0), idx / lap / s_proj (IC 0;
% idx = 0 ilk adimda global arama), n_slow (IC 0). MATLAB Function
% parametreleri (ref, fin, win_*, stop_enable, Ts_ctrl) ayarlanamaz.
al  = {'autorouting', 'smart'};
mfb = 'simulink/User-Defined Functions/MATLAB Function';
ud  = @(blk, ic, pos) add_block('built-in/UnitDelay', blk, 'InitialCondition', ic, ...
    'SampleTime', 'Ts_ctrl', 'Position', pos, 'Orientation', 'left');
zoh = @(blk, pos) add_block('simulink/Discrete/Zero-Order Hold', blk, ...
    'SampleTime', 'Ts_ctrl', 'Position', pos);

% --- Referans ---
rs = [mdl '/Referans'];
add_block('built-in/Subsystem', rs, 'Position', [40 360 180 540]);
add_block(mfb, [rs '/Referans Noktasi'], 'Position', [200 30 360 230]);
set_s2_mfunction([rs '/Referans Noktasi'], refpt_script(), {'ref', 'Ts_ctrl'});
ud([rs '/z_s_ref'], 'ref.s0', [260 280 300 310]);
ro = {'s_ref', 'X_ref', 'Y_ref', 'psi_ref', 'v_ref', 'kappa_ref'};
add_ports(rs, {}, ro, 470);
add_line(rs, 'z_s_ref/1', 'Referans Noktasi/1', al{:});
add_line(rs, 'Referans Noktasi/1', 'z_s_ref/1', al{:});
add_line(rs, 'z_s_ref/1', 's_ref/1', al{:});
for i = 2:6, add_line(rs, sprintf('Referans Noktasi/%d', i), [ro{i} '/1'], al{:}); end
name_line([rs '/Referans Noktasi'], 1, 's_next');
name_line([rs '/z_s_ref'], 1, 's_ref');

% --- Hata Hesabi ---
hs = [mdl '/Hata Hesabi'];
add_block('built-in/Subsystem', hs, 'Position', [300 360 460 600]);
ho = {'e', 's_veh', 's_proj', 'idx', 'lap', 'kappa_p', 'v_ref_p', 's_dot'};
add_ports(hs, {'state', 's_ref'}, ho, 520);
zoh([hs '/ZOH'], [110 35 145 65]);
add_block(mfb, [hs '/Hata'], 'Position', [220 30 380 290]);
set_s2_mfunction([hs '/Hata'], err_script(), {'ref', 'win_fwd', 'win_back'});
% e 1-B vektor (6), log N x 6 olsun (varsayilan [6 1] 6 x 1 x N verir)
ch = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', [hs '/Hata']);
ch.find('-isa', 'Stateflow.Data', 'Name', 'e').Props.Array.Size = '6';
ud([hs '/z_idx'],    '0', [280 330 320 360]);
ud([hs '/z_lap'],    '0', [280 390 320 420]);
ud([hs '/z_s_proj'], '0', [280 450 320 480]);
add_line(hs, 'state/1', 'ZOH/1', al{:});
add_line(hs, 'ZOH/1', 'Hata/1', al{:});
add_line(hs, 's_ref/1', 'Hata/2', al{:});
add_line(hs, 'z_idx/1', 'Hata/3', al{:});
add_line(hs, 'z_lap/1', 'Hata/4', al{:});
add_line(hs, 'z_s_proj/1', 'Hata/5', al{:});
add_line(hs, 'Hata/4', 'z_idx/1', al{:});
add_line(hs, 'Hata/5', 'z_lap/1', al{:});
add_line(hs, 'Hata/3', 'z_s_proj/1', al{:});
for i = 1:numel(ho), add_line(hs, sprintf('Hata/%d', i), [ho{i} '/1'], al{:}); end
for i = 1:numel(ho), name_line([hs '/Hata'], i, ho{i}); end
name_line([hs '/ZOH'], 1, 'state_k');

% --- Bitis ---
bs = [mdl '/Bitis'];
add_block('built-in/Subsystem', bs, 'Position', [580 560 700 640]);
add_ports(bs, {'s_veh', 'state'}, {'done'}, 460);
zoh([bs '/ZOH'], [110 105 145 135]);
add_block(mfb, [bs '/Bitis Mantigi'], 'Position', [220 30 360 170]);
set_s2_mfunction([bs '/Bitis Mantigi'], done_script(), {'stop_enable', 'ref', 'fin', 'Ts_ctrl'});
ud([bs '/z_n_slow'], '0', [270 210 310 240]);
add_block('simulink/Sinks/Stop Simulation', [bs '/Stop'], 'Position', [460 120 490 150]);
add_line(bs, 's_veh/1', 'Bitis Mantigi/1', al{:});
add_line(bs, 'state/1', 'ZOH/1', al{:});
add_line(bs, 'ZOH/1', 'Bitis Mantigi/2', al{:});
add_line(bs, 'z_n_slow/1', 'Bitis Mantigi/3', al{:});
add_line(bs, 'Bitis Mantigi/2', 'z_n_slow/1', al{:});
add_line(bs, 'Bitis Mantigi/1', 'done/1', al{:});
add_line(bs, 'Bitis Mantigi/1', 'Stop/1', al{:});
name_line([bs '/Bitis Mantigi'], 1, 'done');
name_line([bs '/Bitis Mantigi'], 2, 'n_slow');

% --- Ust seviye baglanti ve loglama ---
add_line(mdl, 'Plant/1', 'Hata Hesabi/1', al{:});
add_line(mdl, 'Referans/1', 'Hata Hesabi/2', al{:});
add_line(mdl, 'Hata Hesabi/2', 'Bitis/1', al{:});
add_line(mdl, 'Plant/1', 'Bitis/2', al{:});
add_block('built-in/Mux', [mdl '/Mux_ref'], 'Inputs', '6', 'Position', [240 620 245 740]);
for i = 1:6
    add_line(mdl, sprintf('Referans/%d', i), sprintf('Mux_ref/%d', i), al{:});
    name_line([mdl '/Referans'], i, ro{i});
end
lg = {'Mux_ref', 1, 'ref_pt'; 'Hata Hesabi', 1, 'err'; 'Hata Hesabi', 2, 's_veh';
      'Hata Hesabi', 3, 's_proj'; 'Hata Hesabi', 4, 'idx'; 'Hata Hesabi', 5, 'lap';
      'Hata Hesabi', 6, 'kappa_p'; 'Hata Hesabi', 7, 'v_ref_p'; 'Hata Hesabi', 8, 's_dot';
      'Bitis', 1, 'done'};
for i = 1:size(lg, 1)
    y = 340 + (i - 1)*45;
    add_block('simulink/Sinks/To Workspace', [mdl '/log_' lg{i,3}], ...
        'VariableName', lg{i,3}, tw{:}, 'Position', [800 y 880 y + 24]);
    add_line(mdl, sprintf('%s/%d', lg{i,1}, lg{i,2}), ['log_' lg{i,3} '/1'], al{:});
end
for i = 1:numel(ho), name_line([mdl '/Hata Hesabi'], i, ho{i}); end
name_line([mdl '/Mux_ref'], 1, 'ref_pt');
name_line([mdl '/Bitis'], 1, 'done');
end

function build_s3(mdl, tw)
% S3: Lateral C ve Longitudinal C (ayrik, Ts_ctrl). Her biri: ZOH(state) ->
% MATLAB Function (lat_mpc / lon_mpc, extrinsic) <-> Unit Delay (onceki
% komut, IC 0). ctrl_enable Switch'leri plant girisini secer.
al  = {'autorouting', 'smart'};
mfb = 'simulink/User-Defined Functions/MATLAB Function';
c = {'Lateral C',      {'e', 's_proj', 'state'},          {'delta', 'flag'}, lat_script(), 'mpc_lat', 360;
     'Longitudinal C', {'e', 's_ref', 's_proj', 'state'}, {'F_x', 'flag'},   lon_script(), 'mpc_lon', 200};
for i = 1:2
    sb = [mdl '/' c{i,1}];  ni = numel(c{i,2});
    add_block('built-in/Subsystem', sb, 'Position', [960 c{i,6} 1120 c{i,6} + 100]);
    add_ports(sb, c{i,2}, c{i,3}, 520);
    add_block('simulink/Discrete/Zero-Order Hold', [sb '/ZOH'], 'SampleTime', 'Ts_ctrl', ...
        'Position', [110 35 + 70*(ni-1) 145 65 + 70*(ni-1)]);
    add_block(mfb, [sb '/MPC'], 'Position', [220 30 380 230]);
    set_s2_mfunction([sb '/MPC'], c{i,4}, {'ref', c{i,5}, 'Ts_ctrl'});
    add_block('built-in/UnitDelay', [sb '/z_u'], 'InitialCondition', '0', ...
        'SampleTime', 'Ts_ctrl', 'Position', [280 290 320 320], 'Orientation', 'left');
    for j = 1:ni - 1, add_line(sb, sprintf('%s/1', c{i,2}{j}), sprintf('MPC/%d', j), al{:}); end
    add_line(sb, 'state/1', 'ZOH/1', al{:});
    add_line(sb, 'ZOH/1', sprintf('MPC/%d', ni), al{:});
    add_line(sb, 'z_u/1', sprintf('MPC/%d', ni + 1), al{:});
    add_line(sb, 'MPC/1', 'z_u/1', al{:});
    for j = 1:2, add_line(sb, sprintf('MPC/%d', j), [c{i,3}{j} '/1'], al{:}); end
end
add_line(mdl, 'Hata Hesabi/1', 'Lateral C/1', al{:});
add_line(mdl, 'Hata Hesabi/3', 'Lateral C/2', al{:});
add_line(mdl, 'Plant/1', 'Lateral C/3', al{:});
add_line(mdl, 'Hata Hesabi/1', 'Longitudinal C/1', al{:});
add_line(mdl, 'Referans/1', 'Longitudinal C/2', al{:});
add_line(mdl, 'Hata Hesabi/3', 'Longitudinal C/3', al{:});
add_line(mdl, 'Plant/1', 'Longitudinal C/4', al{:});

% ctrl_enable: 1 -> MPC, 0 -> From Workspace test girisleri
add_block('simulink/Sources/Constant', [mdl '/ctrl_enable'], 'Value', 'ctrl_enable', ...
    'Position', [40 200 100 220]);
sw = {'Sw_Fx', 'Longitudinal C', 'F_x', [180 60 210 100]; 'Sw_delta', 'Lateral C', 'delta', [180 130 210 170]};
for i = 1:2
    add_block('simulink/Signal Routing/Switch', [mdl '/' sw{i,1}], 'Criteria', 'u2 >= Threshold', ...
        'Threshold', '0.5', 'Position', sw{i,4});
    add_line(mdl, [sw{i,2} '/1'], [sw{i,1} '/1'], al{:});
    add_line(mdl, 'ctrl_enable/1', [sw{i,1} '/2'], al{:});
    add_line(mdl, [sw{i,3} '/1'], [sw{i,1} '/3'], al{:});
end
lg = {'Lateral C', 1, 'delta_cmd'; 'Lateral C', 2, 'flag_lat';
      'Longitudinal C', 1, 'Fx_cmd'; 'Longitudinal C', 2, 'flag_lon'};
for i = 1:4
    y = 760 + (i - 1)*45;
    add_block('simulink/Sinks/To Workspace', [mdl '/log_' lg{i,3}], ...
        'VariableName', lg{i,3}, tw{:}, 'Position', [800 y 880 y + 24]);
    add_line(mdl, sprintf('%s/%d', lg{i,1}, lg{i,2}), ['log_' lg{i,3} '/1'], al{:});
    name_line([mdl '/' lg{i,1}], lg{i,2}, lg{i,3});
end
end

function add_ports(sub, ins, outs, xo)
% alt sistem giris / cikis portlari (sirali)
for i = 1:numel(ins)
    y = 43 + (i - 1)*70;
    add_block('built-in/Inport', [sub '/' ins{i}], 'Port', num2str(i), ...
        'Position', [30 y 60 y + 14]);
end
for i = 1:numel(outs)
    y = 43 + (i - 1)*35;
    add_block('built-in/Outport', [sub '/' outs{i}], 'Port', num2str(i), ...
        'Position', [xo y xo + 30 y + 14]);
end
end

function set_s2_mfunction(blk, script, params)
% S2 MATLAB Function: ayrik (Ts_ctrl); parametreler ayarlanamaz (ref gibi
% struct'lar sabit boyutla derlenir).
set_mfunction(blk, script, params);
ch = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', blk);
ch.SampleTime = 'Ts_ctrl';
for k = 1:numel(params)
    d = ch.find('-isa', 'Stateflow.Data', 'Name', params{k});
    d.Tunable = false;
end
% cikislar acikca skaler: kapali dongude (S3) Unit Delay geri beslemeleri
% boyut yayilimini belirsiz birakir (vektor cikislar sonradan ayarlanir)
o = ch.find('-isa', 'Stateflow.Data', 'Scope', 'Output');
for k = 1:numel(o), o(k).Props.Array.Size = '1'; end
end

%% ======================================================================
function build_integrator(sub, in_names, out_names, ics, lower)
% Ayri surekli Integrator'lar; lower{i} bos degilse alt sinir (v_x >= 0).
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
function s = lt_script()
s = strjoin({
'function [F_zf, F_zr] = fcn(a_x, m, g, lf, lr, L, h)'
'% Yuk Transferi: aks dusey yukleri (>= 0); a_x govde boyuna ivmesi,'
'% Memory a_x uzerinden bir adim (Ts_plant) gecikmeli.'
'F_zf = max((m*g*lr - m*a_x*h)/L, 0);'
'F_zr = max((m*g*lf + m*a_x*h)/L, 0);'
}, newline);
end

function s = tyr_script()
s = strjoin({
'function [F_xf, F_yf, F_xr, F_yr] = fcn(F_x, delta, v_x, v_y, r, F_zf, F_zr, lf, lr, Cf, Cr, mu, vx_floor, brake_split)'
'% Lastik Modeli: kuvvetler teker cercevesinde, aks basina (iki teker toplu).'
'% Itis yalniz arkadan; fren brake_split ile on/arka.'
'if F_x >= 0'
'    F_xf = 0;'
'    F_xr = F_x;'
'else'
'    F_xf = brake_split*F_x;'
'    F_xr = (1 - brake_split)*F_x;'
'end'
'% surtunme cemberi: once boyuna kirpilir, kalan pay yanal tavandir'
'F_xf = min(max(F_xf, -mu*F_zf), mu*F_zf);'
'F_xr = min(max(F_xr, -mu*F_zr), mu*F_zr);'
'% kayma acilari (S1 notlar), bolme tabani vx_floor'
'vx_hat = max(v_x, vx_floor);'
'a_f = delta - atan((v_y + lf*r)/vx_hat);'
'a_r = -atan((v_y - lr*r)/vx_hat);'
'F_yf = dugoff(a_f, Cf, sqrt(max((mu*F_zf)^2 - F_xf^2, 0)));'
'F_yr = dugoff(a_r, Cr, sqrt(max((mu*F_zr)^2 - F_xr^2, 0)));'
'end'
''
'function F = dugoff(a, C, Fmax)'
'% Yanal Dugoff (Rajamani 13, kappa yok), S = C|tan a|: 2S <= Fmax dogrusal,'
'% degilse doymus. Fmax = 0 iken F = 0 (S = 0 dogrusal kol, S > 0 doymus kol).'
'S = C*abs(tan(a));'
'if 2*S <= Fmax'
'    F = C*tan(a);'
'else'
'    F = sign(a)*Fmax*(1 - Fmax/(4*S));'
'end'
'end'
}, newline);
end

function s = dyn_script()
s = strjoin({
'function [vx_dot, vy_dot, r_dot, a_x] = fcn(F_xf, F_yf, F_xr, F_yr, delta, v_x, v_y, r, m, Iz, lf, lr, rho_air, CdA, C_rr, g, theta_road)'
'% Dinamik Denklem: tam form govde denklemleri (Rajamani 2.3), lastik'
'% kuvvetleri (teker cercevesi, aks basina) Lastik Modeli''nden gelir.'
'% Direnc: hava + yuvarlanma + egim (S1; v_x >= 0).'
'F_res = 0.5*rho_air*CdA*v_x^2 + C_rr*m*g*cos(theta_road) + m*g*sin(theta_road);'
'vx_dot = (F_xf*cos(delta) - F_yf*sin(delta) + F_xr - F_res)/m + v_y*r;'
'vy_dot = (F_yf*cos(delta) + F_xf*sin(delta) + F_yr)/m - v_x*r;'
'r_dot  = (lf*(F_yf*cos(delta) + F_xf*sin(delta)) - lr*F_yr)/Iz;'
'a_x    = vx_dot - v_y*r;   % govde boyuna ivmesi (yuk transferi icin)'
}, newline);
end

function s = refpt_script()
s = strjoin({
'function [s_next, X_ref, Y_ref, psi_ref, v_ref, kappa_ref] = fcn(s_ref, ref, Ts_ctrl)'
'% Referans Noktasi: s_ref[k+1] = s_ref[k] + v_ref(s_ref[k] mod L) Ts_ctrl'
'% (ref_advance); s_ref noktasinda yolun konumu, yonu, hizi ve egriligi'
'% (lineer ara deger, yon sarma farkiyla).'
's_next    = ref_advance(s_ref, ref, Ts_ctrl);'
'X_ref     = periodic_interp(ref.x, s_ref, ref.ds);'
'Y_ref     = periodic_interp(ref.y, s_ref, ref.ds);'
'v_ref     = periodic_interp(ref.v, s_ref, ref.ds);'
'kappa_ref = periodic_interp(ref.kappa, s_ref, ref.ds);'
'N = numel(ref.theta);'
'u = s_ref/ref.ds;  j = floor(u);  t = u - j;'
'a = mod(j, N) + 1;  b = mod(j + 1, N) + 1;'
'dth = mod(ref.theta(b) - ref.theta(a) + pi, 2*pi) - pi;'
'psi_ref = mod(ref.theta(a) + t*dth + pi, 2*pi) - pi;'
}, newline);
end

function s = err_script()
s = strjoin({
'function [e, s_veh, s_proj, idx, lap, kappa_p, v_ref_p, s_dot] = fcn(x, s_ref, idx_prev, lap_prev, s_proj_prev, ref, win_fwd, win_back)'
'% Hata: x = [X Y psi v_x v_y r] (ZOH), s_ref acilmis. ref_error (Rajamani 2.5).'
'% e = [e_x; e_y; e_psi; e_x_dot; e_y_dot; e_psi_dot].'
'[e_x, e_y, e_psi, e_x_dot, e_y_dot, e_psi_dot, s_veh, s_proj, idx, lap, ~, v_ref_p, kappa_p, ~, s_dot] = ...'
'    ref_error(x(1), x(2), x(3), x(4), x(5), x(6), s_ref, idx_prev, lap_prev, s_proj_prev, ...'
'              ref, win_fwd, win_back);'
'e = [e_x; e_y; e_psi; e_x_dot; e_y_dot; e_psi_dot];'
}, newline);
end

function s = done_script()
s = strjoin({
'function [done, n_slow] = fcn(s_veh, x, n_slow_prev, stop_enable, ref, fin, Ts_ctrl)'
'% Bitis Mantigi: tur (s_veh - s0 >= n_laps L) ya da t_hold boyunca'
'% hypot(v_x, v_y) < v_stop; stop_enable = 0 iken hic bitmez (ref_done).'
'[done, n_slow] = ref_done(s_veh, x(4), x(5), n_slow_prev, stop_enable, ref, fin, Ts_ctrl);'
}, newline);
end

function s = lat_script()
s = strjoin({
'function [delta, flag] = fcn(e, s_proj, x, delta_prev, ref, mpc_lat, Ts_ctrl)'
'% Lateral C: yanal MPC (lat_mpc, MATLAB''da calisir; quadprog).'
'coder.extrinsic(''lat_mpc'');'
'delta = 0;  flag = 0;'
'[delta, flag] = lat_mpc(e, s_proj, x(4), delta_prev, ref, mpc_lat, Ts_ctrl);'
}, newline);
end

function s = lon_script()
s = strjoin({
'function [F_x, flag] = fcn(e, s_ref, s_proj, x, F_prev, ref, mpc_lon, Ts_ctrl)'
'% Longitudinal C: boyuna MPC (lon_mpc, MATLAB''da calisir; quadprog).'
'coder.extrinsic(''lon_mpc'');'
'F_x = 0;  flag = 0;'
'[F_x, flag] = lon_mpc(e(1), s_ref, s_proj, x(4), x(4)*x(6), F_prev, ref, mpc_lon, Ts_ctrl);'
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
