function val = periodic_interp(tab, s, ds)
%PERIODIC_INTERP  Periyodik tabloda lineer ara deger (S2).
%   val = periodic_interp(tab, s, ds): tab(k) degeri s = (k-1)*ds noktasinda;
%   yol kapali, tab(N) -> tab(1) arasi da bir ds. s herhangi bir gercek
%   sayi olabilir (acilmis s dahil), indeks mod N ile sarilir.
%#codegen
N = numel(tab);
u = s / ds;
j = floor(u);
t = u - j;
a = mod(j, N) + 1;
b = mod(j + 1, N) + 1;
val = tab(a) + t * (tab(b) - tab(a));
end
