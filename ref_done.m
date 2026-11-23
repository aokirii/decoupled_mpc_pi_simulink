function [done, n_slow] = ref_done(s_veh, v_x, v_y, n_slow_prev, stop_enable, ref, fin, Ts)
%REF_DONE  Bitis mantigi (S2).
%   [done, n_slow] = ref_done(s_veh, v_x, v_y, n_slow_prev, stop_enable, ref, fin, Ts)
%   fin: n_laps, v_stop, t_hold. Ts: ornekleme suresi (Simulink'te Ts_ctrl;
%   ref_advance ile ayni tek kaynak).
%   n_slow : hypot(v_x, v_y) < v_stop olan ardisik ornek sayisi (geri
%            besleme, baslangic 0).
%   done = stop_enable VE ( s_veh - s0 >= n_laps*L
%                           VEYA (n_slow - 1)*Ts >= t_hold ).
%   Ilk yavas ornek k0 ise durma olcutu k0 + t_hold/Ts adiminda tetiklenir
%   (arac en az t_hold boyunca kesintisiz yavas). stop_enable = 0 iken done
%   hic 1 olmaz; n_slow yine sayilir.
%#codegen
if hypot(v_x, v_y) < fin.v_stop
    n_slow = n_slow_prev + 1;
else
    n_slow = 0;
end
lap_ok  = s_veh - ref.s0 >= fin.n_laps * ref.L;
stop_ok = n_slow >= 1 && (n_slow - 1) * Ts >= fin.t_hold - 1e-9;
done = double(stop_enable ~= 0 && (lap_ok || stop_ok));
end
