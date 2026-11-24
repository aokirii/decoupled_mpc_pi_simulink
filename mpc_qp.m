function [u, flag] = mpc_qp(Ac, Bc, Ec, w, x0, Ts, p, u_prev, xmax)
%MPC_QP  Ortak yogunlastirilmis dogrusal MPC (S3; lat_mpc ve lon_mpc kullanir).
%   x' = Ac x + Bc u + Ec w, ZOH ile Ts'de ayriklastirilir (expm).
%   w (nw x Np) bilinen bozucu onizlemesi. Durum sifira regule edilir:
%     min sum_k x_k' Q x_k + R u_k^2 + Rd (u_k - u_{k-1})^2 + ws sg^2
%     ulim(1) <= u_k <= ulim(2), |u_k - u_{k-1}| <= dumax,
%     x_k(isoft) <= xmax_k + sg (isoft = 0: yumusak sinir yok).
%   p: Q, R, Rd, ulim, dumax, isoft, ws. Cikis: ilk giris u ve quadprog
%   exitflag. Basarisizlikta (bos cozum ya da exitflag < 0) u = u_prev.
nx = numel(x0);  Np = size(w, 2);  nw = size(w, 1);
M  = expm([Ac, Bc, Ec; zeros(1 + nw, nx + 1 + nw)] * Ts);
A  = M(1:nx, 1:nx);  B = M(1:nx, nx+1);  E = M(1:nx, nx+2:end);
G = zeros(nx*Np, Np);  F = zeros(nx*Np, 1);  f = x0(:);  g = zeros(nx, Np);
for k = 1:Np
    r = (k-1)*nx + (1:nx);
    f = A*f + E*w(:,k);  g = A*g;  g(:,k) = B;
    F(r) = f;  G(r,:) = g;
end
Qb = kron(eye(Np), p.Q);
D  = eye(Np) - diag(ones(Np-1, 1), -1);  e1 = [1; zeros(Np-1, 1)];
H  = blkdiag(G'*Qb*G + p.R*eye(Np) + p.Rd*(D'*D), p.ws);
h  = [G'*Qb*F - p.Rd*u_prev*D'*e1; 0];
Ai = [D, zeros(Np,1); -D, zeros(Np,1)];
bi = [p.dumax + u_prev*e1; p.dumax - u_prev*e1];
if p.isoft > 0
    Ai = [Ai; G(p.isoft:nx:end, :), -ones(Np,1)];
    bi = [bi; xmax(:) - F(p.isoft:nx:end)];
end
lb = [p.ulim(1)*ones(Np,1); 0];  ub = [p.ulim(2)*ones(Np,1); inf];
opt = optimoptions('quadprog', 'Algorithm', 'active-set', 'Display', 'off');
z0 = [min(max(u_prev, p.ulim(1)), p.ulim(2))*ones(Np,1); 0];   % active-set icin sinir ici baslangic
[z, ~, flag] = quadprog((H + H')/2, h, Ai, bi, [], [], lb, ub, z0, opt);
if isempty(z) || flag < 0, u = u_prev; else, u = z(1); end   % uygulanamaz/hata: onceki komut
end
