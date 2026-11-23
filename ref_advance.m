function s_next = ref_advance(s_ref, ref, Ts)
%REF_ADVANCE  Hareketli referans noktasini bir adim ilerletir (S2).
%   s_next = ref_advance(s_ref, ref, Ts)
%   s_ref[k+1] = s_ref[k] + v_ref(s_ref[k] mod L) * Ts, v_ref lineer ara
%   degerli (periodic_interp). s_ref acilmistir (tur sonrasi L'yi asar);
%   baslangic s_ref(0) = ref.s0 cagiran tarafta (Unit Delay IC) verilir.
%#codegen
s_next = s_ref + periodic_interp(ref.v, s_ref, ref.ds) * Ts;
end
