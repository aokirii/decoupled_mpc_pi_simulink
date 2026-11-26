function Gorsellestirme(out)
%GORSELLESTIRME  VehicleDiagram simulasyonu sonrasi 3B tekrar oynatici (S4).
%   Model_Setup; out = sim('VehicleDiagram'); Gorsellestirme(out)
%   * Zaman ekseni Ts_ctrl (out.err.Time); surekli durumlar (X, Y, psi,
%     v_x, v_y, r) bu eksene interp1 ile alinir. ref, m, lf, lr, vx_floor
%     base workspace'ten okunur (model de oradan okur).
%   * delta = delta_cmd, a_cmd = Fx_cmd/m; a_x, F_yf, F_yr logsout'tan;
%     a_y = vy_dot + v_x r; alpha_f, alpha_r plant'taki Lastik Modeli
%     formulu (v_hat = max(v_x, vx_floor)).
%   * Arac modeli MATLAB kurulumundaki Car_Mustang.x3d dosyasindan calisma
%     aninda okunur (Simulink 3D Animation ile gelir; HUMUSOFT/MathWorks
%     telif hakli, depoya kopyalanmaz). Dosya yoksa ya da bicimi beklenmeyen
%     ise basit kutu cizilir. Yalnizca temel MATLAB kullanilir.
%   * Katmanlar: arac ustunde yon oku (siyah) ve hiz oku (turuncu; aradaki
%     aci beta), aks yanal kuvvet oklari (on: camgobegi, arka: sari);
%     |e_y| ile renklenen iz + renk cubugu; HUD; sagda zaman serileri
%     paneli (kirmizi zaman imleci); takip kamerasi kapaliyken arac
%     konumunu gosteren dikey isaret.
%   * Kontroller: zaman kaydirici (suruklerken canli), Oynat/Duraklat,
%     hiz (1x-20x gercek zaman), takip kamerasi (acikken arac arkadan
%     izlenir; kapaliyken tum harita gorunur ve fare ile dondurulebilir).

    %% --- Veri (loglar Ts_ctrl eksenine) ---
    ref = evalin('base', 'ref');   m  = evalin('base', 'm');
    lf  = evalin('base', 'lf');    lr = evalin('base', 'lr');
    vfl = evalin('base', 'vx_floor');
    t  = out.err.Time;  N = numel(t);
    if N < 2, error('Gorsellestirme: en az 2 ornek gerekli (N = %d)', N); end
    Ts = (t(end) - t(1)) / (N - 1);
    ip = @(s) interp1(s.Time, s.Data, t);
    lg = @(n) ip(out.logsout.get(n).Values);
    xs = ip(out.X);    ys = ip(out.Y);    th = ip(out.psi);
    vx = ip(out.v_x);  vy = ip(out.v_y);  r  = ip(out.r);
    de = out.delta_cmd.Data;  ey = out.err.Data(:,2);  eh = out.err.Data(:,3);
    vref = out.v_ref_p.Data;  acmd = out.Fx_cmd.Data / m;  a_x = lg('a_x');
    ay  = lg('vy_dot') + vx.*r;  Fyf = lg('F_yf');  Fyr = lg('F_yr');
    vh  = max(vx, vfl);
    af  = de - atan((vy + lf*r) ./ vh);  ar = -atan((vy - lr*r) ./ vh);
    beta = atan2(vy, vx);  beta(hypot(vx, vy) < 0.05) = NaN;   % durgunken beta tanimsiz
    x_ref = ref.x([1:end 1]);  y_ref = ref.y([1:end 1]);       % kapali dongu
    cF = [0 0.75 0.9];  cR = [0.9 0.7 0];       % on / arka aks kuvvet renkleri

    %% --- Figure ve 3B sahne (z-up dunya, metre) ---
    pad  = 30;
    xl   = [min([x_ref; xs]) - pad, max([x_ref; xs]) + pad];
    yl   = [min([y_ref; ys]) - pad, max([y_ref; ys]) + pad];
    span = max(diff(xl), diff(yl));

    fig = figure('Color','w', 'Name','VehicleDiagram 3D: iki MPC, sade plant', 'Position',[60 60 1400 800]);
    if isprop(fig, 'Theme'), fig.Theme = 'light'; end      % R2025a+; eski surumde atla
    pnl = uipanel('Parent',fig, 'Units','normalized', 'Position',[0.01 0.10 0.63 0.79], ...
                  'BorderType','none', 'BackgroundColor','w');   % sahneyi sol alana kirpar
    ax  = axes('Parent',pnl, 'Position',[0 0 1 1]);
    hold(ax,'on');  axis(ax,'off');  daspect(ax,[1 1 1]);
    xlim(ax, xl);  ylim(ax, yl);  zlim(ax, [-1, max(3, 0.2*span)]);
    set(ax, 'Projection','perspective');
    disableDefaultInteractivity(ax);

    % Zemin ve yol seridi (yarim genislik 3.5 m, kenarlar yol tegetinden).
    % Zemin yolun 0.5 m altinda: uzak genel bakista derinlik tamponu 5 cm'yi
    % ayiramiyor, zemin yolun icinden gorunuyordu (z-fighting).
    patch(ax, 'XData',xl([1 2 2 1]), 'YData',yl([1 1 2 2]), ...
          'ZData',-0.5*ones(1,4), 'FaceColor',[0.88 0.92 0.85], ...
          'EdgeColor','none', 'FaceLighting','none');
    tx = gradient(x_ref);  ty = gradient(y_ref);  nrm = hypot(tx, ty);
    nx = -ty ./ nrm;  ny = tx ./ nrm;  hw = 3.5;
    surf(ax, [x_ref + hw*nx, x_ref - hw*nx]', [y_ref + hw*ny, y_ref - hw*ny]', ...
         zeros(2, numel(x_ref)), 'FaceColor',[0.45 0.45 0.45], ...
         'EdgeColor','none', 'FaceLighting','none');

    % Referans, start (ilk konum) / goal (son konum)
    plot3(ax, x_ref, y_ref, 0.03 + 0*x_ref, 'b--', 'LineWidth',1.5);
    plot3(ax, xs(1), ys(1), 0.1, 'go', 'MarkerSize',10, ...
          'LineWidth',2, 'MarkerFaceColor','g');
    plot3(ax, xs(end), ys(end), 0.1, 'rx', 'MarkerSize',12, 'LineWidth',2);

    % |e_y| ile renklenen iz: NaN ile biten patch cizgisi, sabit olcek [0 0.5] m
    hTrail = patch(ax, 'XData',[xs(1); nan], 'YData',[ys(1); nan], ...
                   'ZData',[0.06; nan], 'CData',[abs(ey(1)); nan], ...
                   'EdgeColor','interp', 'FaceColor','none', 'LineWidth',2.5);
    cmap = interp1([0 0.5 1], [0.1 0.8 0.1; 0.95 0.85 0.1; 0.9 0.1 0.1], linspace(0, 1, 64)');
    colormap(ax, cmap);  clim(ax, [0 0.5]);           % 0.5 m ustu kirmiziya doyar
    % Renk cubugu panel disinda (HUD'un saginda): gorunmez yardimci eksen uzerinden
    axc = axes('Parent',fig, 'Position',[0.50 0.925 0.12 0.016], 'Visible','off');
    colormap(axc, cmap);  clim(axc, [0 0.5]);  disableDefaultInteractivity(axc);
    cb = colorbar(axc, 'northoutside');
    set(cb, 'Position',[0.50 0.925 0.12 0.016], 'AxisLocation','in');   % etiket ustte, sayilar altta
    cb.Label.String = '|e_y| [m]';

    % Takip kamerasi kapaliyken arac konumu: dikey cubuk + tepe isareti
    hStem = plot3(ax, nan(1,2), nan(1,2), [0 0.06*span], '-o', ...
                  'Color',[0.85 0 0.85], 'LineWidth',2, 'MarkerIndices',2, ...
                  'MarkerSize',12, 'MarkerFaceColor',[0.85 0 0.85], ...
                  'MarkerEdgeColor','k');

    % Arac: yerel cerceve ileri=+x, sol=+y, yukari=+z; poz hgtransform ile
    [V, F, C, lit] = load_car();
    hCar = hgtransform('Parent', ax);
    patch('Parent',hCar, 'Vertices',V, 'Faces',F, 'FaceVertexCData',C, ...
          'FaceColor','flat', 'EdgeColor','none', 'FaceLighting',lit, ...
          'AmbientStrength',0.45, 'SpecularStrength',0.3);
    light(ax, 'Style','infinite', 'Position',[1 -2 3]);
    light(ax, 'Style','infinite', 'Position',[-2 1 1], 'Color',[0.35 0.35 0.35]);

    % Oklar: arac yerel cercevesinde, hgtransform altinda (poz otomatik),
    % cati ustunde z = 1.7 m. Yon oku sabit; hiz ve kuvvet oklari show() ile
    zA = 1.7;
    [X, Y, Z] = arrow_xyz([0 0], [6 0], zA);
    line('Parent',hCar, 'XData',X, 'YData',Y, 'ZData',Z, 'Color','k', 'LineWidth',2);
    hVel = line('Parent',hCar, 'XData',nan, 'YData',nan, 'ZData',nan, ...
                'Color',[1 0.55 0], 'LineWidth',2.5);
    aF  = sort(abs([Fyf; Fyr]));                           % iki aks birlikte
    sF  = 4 / max(aF(ceil(0.95*numel(aF))), 100);          % %95 yuzdelik -> 4 m
    fyA = @(F) sign(F) * min(abs(F)*sF, 6);                % ok boyu en fazla 6 m
    hFyf = line('Parent',hCar, 'XData',nan, 'YData',nan, 'ZData',nan, ...
                'Color',cF, 'LineWidth',2.5);
    hFyr = line('Parent',hCar, 'XData',nan, 'YData',nan, 'ZData',nan, ...
                'Color',cR, 'LineWidth',2.5);

    %% --- Zaman serileri paneli (sag; tam seri bir kez cizilir, imlec guncellenir) ---
    S = {[vx vref], 'v_x, v_{ref} [m/s]'; rad2deg(de), 'delta [deg]'; ey, 'e_y [m]'; ...
         rad2deg(beta), 'beta [deg]'; rad2deg(r), 'r [deg/s]'; [Fyf Fyr]/1000, 'F_y [kN]'};
    nP  = size(S, 1);  hP = (0.86 - 0.03*(nP-1)) / nP;
    pax = gobjects(nP, 1);  hCur = gobjects(nP, 1);
    for i = 1:nP
        pax(i) = axes('Parent',fig, 'Position',[0.71, 0.95 - i*hP - 0.03*(i-1), 0.27, hP]);
        hl = plot(pax(i), t, S{i,1}, 'LineWidth',1);
        grid(pax(i), 'on');  ylabel(pax(i), S{i,2});
        xlim(pax(i), [t(1), t(end)]);
        hCur(i) = xline(pax(i), t(1), 'r-', 'LineWidth',1);
    end
    set(pax(1:end-1), 'XTickLabel',[]);  xlabel(pax(end), 't [s]');
    linkaxes(pax, 'x');
    set(hl(1), 'Color',cF);  set(hl(2), 'Color',cR);
    legend(hl, {'F_{yf}','F_{yr}'}, 'Location','best');

    %% --- HUD ve kontroller ---
    hHud = uicontrol(fig, 'Style','text', 'Units','normalized', ...
                     'Position',[0.02 0.895 0.46 0.095], 'BackgroundColor','w', ...
                     'ForegroundColor','k', 'FontName','FixedWidth', ...
                     'FontSize',8, 'HorizontalAlignment','left');
    hPlay = uicontrol(fig, 'Style','togglebutton', 'String','Oynat', ...
                      'Units','normalized', 'Position',[0.02 0.025 0.07 0.05], ...
                      'Callback',@on_play);
    speeds = [1 5 10 20];
    hSpeed = uicontrol(fig, 'Style','popupmenu', 'String',{'1x','5x','10x','20x'}, ...
                       'Units','normalized', 'Position',[0.10 0.025 0.06 0.05], ...
                       'Callback',@on_resync);
    hSlider = uicontrol(fig, 'Style','slider', 'Min',1, 'Max',N, 'Value',1, ...
                        'SliderStep',[1 10]/(N-1), 'Units','normalized', ...
                        'Position',[0.17 0.03 0.35 0.04], 'Callback',@on_slider);
    addlistener(hSlider, 'ContinuousValueChange', @on_slider);   % surukleme sirasinda canli
    hFollow = uicontrol(fig, 'Style','checkbox', 'String','Takip kamerasi', ...
                        'Value',1, 'Units','normalized', 'BackgroundColor','w', ...
                        'ForegroundColor','k', 'Position',[0.53 0.025 0.11 0.05], ...
                        'Callback',@on_follow);
    hRot = rotate3d(fig);
    setAllowAxesRotate(hRot, [pax; axc], false);   % yalniz 3B sahne donsun

    k0 = 1;  t_start = tic;        % oynatma zaman referansi
    playing = false;  close_req = false;
    set(fig, 'CloseRequestFcn', @on_close);   % oynatma sirasinda kapatma ertelenir
    on_follow();  show(1);

    %% --- Ic fonksiyonlar (paylasilan durum) ---
    function show(k)
        k = min(max(round(k), 1), N);
        set(hCar, 'Matrix', makehgtform('translate',[xs(k) ys(k) 0], ...
                                         'zrotate',th(k)));
        set(hTrail, 'XData',[xs(1:k); nan], 'YData',[ys(1:k); nan], ...
                    'ZData',[0.06 + 0*xs(1:k); nan], 'CData',[abs(ey(1:k)); nan]);
        set(hStem, 'XData',[xs(k) xs(k)], 'YData',[ys(k) ys(k)]);
        [X, Y, Z] = arrow_xyz([0 0], 1.2*[vx(k) vy(k)], zA);   % 5 m/s -> 6 m
        set(hVel, 'XData',X, 'YData',Y, 'ZData',Z);
        [X, Y, Z] = arrow_xyz([lf 0], [0 fyA(Fyf(k))], zA);
        set(hFyf, 'XData',X, 'YData',Y, 'ZData',Z);
        [X, Y, Z] = arrow_xyz([-lr 0], [0 fyA(Fyr(k))], zA);
        set(hFyr, 'XData',X, 'YData',Y, 'ZData',Z);
        set(hHud, 'String', { ...
            sprintf(['t = %7.2f s   delta = %6.2f deg   e_y = %7.3f m   ' ...
                     'e_psi = %6.2f deg'], t(k), rad2deg(de(k)), ey(k), rad2deg(eh(k)));
            sprintf('v_x = %5.2f m/s (%5.1f km/h)   v_y = %6.3f m/s   r = %7.2f deg/s', ...
                    vx(k), 3.6*vx(k), vy(k), rad2deg(r(k)));
            sprintf('beta = %6.2f deg   a_y = %6.2f m/s^2 (%4.2f g)', ...
                    rad2deg(beta(k)), ay(k), ay(k)/9.81);
            sprintf(['alpha_f = %6.2f deg   alpha_r = %6.2f deg   ' ...
                     'F_yf = %7.2f kN   F_yr = %7.2f kN'], ...
                    rad2deg(af(k)), rad2deg(ar(k)), Fyf(k)/1000, Fyr(k)/1000);
            sprintf('v_ref = %5.2f m/s   a_x = %6.2f m/s^2   a_cmd = %6.2f m/s^2', ...
                    vref(k), a_x(k), acmd(k))});
        set(hCur, 'Value', t(k));
        if get(hFollow,'Value'), follow_cam(k); end
    end

    function follow_cam(k)
        % Aracin ~15 m arkasi, 6 m yukarisi; birkac metre onune bakar
        d = [cos(th(k)) sin(th(k))];
        set(ax, 'CameraPosition',[xs(k) - 15*d(1), ys(k) - 15*d(2), 6], ...
                'CameraTarget',  [xs(k) +  5*d(1), ys(k) +  5*d(2), 1], ...
                'CameraUpVector',[0 0 1], 'CameraViewAngle',40);
    end

    function on_follow(~, ~)
        if get(hFollow,'Value')
            hRot.Enable = 'off';  set(hStem, 'Visible','off');
            show(get(hSlider,'Value'));
        else
            % Genel bakis: tum harita egik acidan, fare ile dondurulebilir
            c = [mean(xl) mean(yl) 0];
            set(ax, 'CameraTarget',c, 'CameraPosition',c + span*[-0.7 -0.9 0.8], ...
                    'CameraUpVector',[0 0 1], 'CameraViewAngleMode','auto');
            hRot.Enable = 'on';  set(hStem, 'Visible','on');
        end
    end

    function on_slider(~, ~)
        on_resync();
        show(k0);
    end

    function on_resync(~, ~)
        % Kaydirici/hiz degisince zaman referansini sifirla (sicrama olmasin)
        k0 = round(get(hSlider,'Value'));  t_start = tic;
    end

    function on_play(~, ~)
        if ~get(hPlay,'Value') || playing, return; end   % Duraklat / zaten oynuyor
        playing = true;  cleanup = onCleanup(@stop_play);   % cikista (hata dahil)
        on_resync();
        if k0 >= N, k0 = 1; t_start = tic; end    % sondaysa bastan oynat
        set(hPlay, 'String','Duraklat');
        while ishandle(fig) && get(hPlay,'Value')
            k = k0 + floor(toc(t_start) * speeds(get(hSpeed,'Value')) / Ts);
            if k >= N, k = N; set(hPlay,'Value',0); end
            set(hSlider, 'Value',k);  show(k);
            drawnow;                 % kapatma istegi burada on_close'a gider, silme ertelenir
        end
    end

    function stop_play()
        % Oynatma bitti (duraklat, log sonu ya da hata): dugmeyi sifirla,
        % ertelenmis kapatma varsa figure'u simdi sil (on_play donerken calisir,
        % ana calisma alani yasiyor)
        playing = false;   %#ok<MOCUP>
        if ishandle(hPlay), set(hPlay, 'Value',0, 'String','Oynat'); end
        if close_req && ishandle(fig), delete(fig); end
    end

    function on_close(~, ~)
        % X dugmesi / close / close all: oynatirken silme, durdur ve ertele
        if playing, close_req = true;  set(hPlay, 'Value',0);
        else,       delete(fig);
        end
    end
end

function [X, Y, Z] = arrow_xyz(p0, v, z)
% Ok: sap + iki uc kolu tek cizgi olarak (p0 -> uc -> kol1 -> uc -> kol2), yukseklik z
    n = hypot(v(1), v(2));
    if n < 1e-6, X = nan; Y = nan; Z = nan; return; end
    u = v / n;  h = min(0.8, 0.35*n);  w = [-u(2) u(1)];     % uc kolu, dik yon
    tip = p0 + v;
    P = [p0; tip; tip + h*(-cos(pi/6)*u + sin(pi/6)*w); tip; ...
         tip + h*(-cos(pi/6)*u - sin(pi/6)*w)];
    X = P(:,1);  Y = P(:,2);  Z = z + 0*X;
end

%% --- Arac modeli ---
function [V, F, C, lit] = load_car()
% Car_Mustang.x3d varsa onu okur (lit = 'gouraud'); dosya yoksa veya bicimi
% beklenmeyen ise 4.6 x 1.8 x 1.3 m kutu dondurur (lit = 'flat').
    f = fullfile(matlabroot, 'toolbox','sl3d','library','objects', ...
                 'Components','Vehicles','Car_Mustang.x3d');
    lit = 'gouraud';
    if exist(f, 'file') ~= 2
        fprintf('Gorsellestirme: Car_Mustang.x3d bulunamadi, kutu model kullaniliyor.\n');
    else
        try
            [V, F, C] = parse_mustang(f);
            return
        catch
            fprintf('Gorsellestirme: Car_Mustang.x3d beklenmeyen bicimde, kutu model kullaniliyor.\n');
        end
    end
    % Kutu model: duz isiklandirma ve acik renk ile net okunur
    [bx, by, bz] = ndgrid([-2.3 2.3], [-0.9 0.9], [0 1.3]);
    V = [bx(:) by(:) bz(:)];
    F = [1 2 4 3; 5 6 8 7; 1 2 6 5; 3 4 8 7; 1 3 7 5; 2 4 8 6];
    C = repmat([0.55 0.65 0.95], 6, 1);
    lit = 'flat';
end

function [V, F, C] = parse_mustang(f)
% Car_Mustang.x3d'yi metin olarak okur; yerel cerceveye (ileri=+x, sol=+y,
% yukari=+z) cevirir, uzunlugu 4.6 m yapar, tabani z=0'a oturtur.
% X3D dosyasi Y-up: uzunluk Z ekseninde (+Z on), genislik X, yukseklik Y.
    txt = fileread(f);
    P   = x3d_nums(txt, '<Coordinate[^>]*\spoint=''([^'']*)''');
    idx = x3d_nums(txt, '\scoordIndex=''([^'']*)''');    % normalIndex/colorIndex degil
    cix = x3d_nums(txt, '\scolorIndex=''([^'']*)''');
    col = x3d_nums(txt, '<Color[^>]*\scolor=''([^'']*)''');

    % Eksen esleme (donel permutasyon, el yonu korunur), olcek, yere oturtma
    V = reshape(P, 3, []).';
    V = V(:, [3 1 2]);
    V = V * (4.6 / (max(V(:,1)) - min(V(:,1))));
    V(:,1:2) = V(:,1:2) - (max(V(:,1:2)) + min(V(:,1:2))) / 2;
    V(:,3)   = V(:,3) - min(V(:,3));

    % -1 ile ayrilmis poligonlar (3..16 kose) -> NaN dolgulu Faces matrisi
    if idx(end) ~= -1, idx(end+1) = -1; end
    cut   = find(idx == -1);
    first = [1; cut(1:end-1) + 1];
    len   = cut - first;
    F = nan(numel(cut), max(len));
    for i = 1:numel(cut)
        F(i, 1:len(i)) = idx(first(i):cut(i)-1).' + 1;
    end
    C = reshape(col, 3, []).';
    C = C(cix + 1, :);                 % yuz basina bir renk indeksi
    assert(size(C,1) == size(F,1) && max(F(:)) <= size(V,1));   % tutarsizsa kutuya dus
end

function v = x3d_nums(txt, pat)
% Tek bir X3D ozniteligindeki sayilari sutun vektoru olarak dondurur
    tok = regexp(txt, pat, 'tokens', 'once');
    v   = sscanf(strrep(tok{1}, ',', ' '), '%f');
end
