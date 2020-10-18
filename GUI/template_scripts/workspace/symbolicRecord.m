%%
clear all
%%
syms u v t

syms c1 c2 c3 c4 c5 c6 c7 c8 c9 c10 c11 c12 c13 c14 c15 c16 c17 c18 c19 c20 c21 c22 c23 c24 c25 c26 c27 c28
syms b11 b12 b13 b21 b22 b23 b31 b32 b33
syms A1 A2 A3 B1 B2 B3
B = sym('b%d%d',[3,3]);
OA = sym('A%d',[3,1]);
c = sym('c%d',[31,1]);
C = sym('c%d',[31,1]);
%% implicit surface 
Fi = @(x,y,z) c(1).*x.^4 + c(2).*y.^4 + c(3).*z.^4 +...
                    c(4).*x.^3.*y + c(5).*x.^3.*z + c(6).*y.^3.*x + c(7).*y.^3.*z + c(8).*z.^3.*x + c(9).*z.^3.*y + ...
                    c(10)*x.^2.*y^2 + c(11)*x.^2.*z^2 + c(12)*y.^2.*z^2 + ...
                    c(13).*x.^3 + c(14).*y.^3 + c(15).*z.^3 +...
                    c(16).*x.^2.*y + c(17).*x.^2.*z + c(18).*y.^2.*x + c(19).*y.^2.*z +...
                    c(20).*z.^2.*x + c(21).*z.^2.*y + ...
                    c(22).*x.^2 + c(23).*y.^2 + c(24).*z.^2 +...
                    c(25).*x.*y + c(26).*x.*z + c(27).*y.*z + ...
                    c(28).*x + c(29).*y + c(30).*z + c(31);

%% linear surface tangential intersection
B = sym('b%d',[3,2]);
OBu = @(u) [b11.*u + b12;
    b21.*u + b22;
    b31.*u + b32];

%% curve OB(u)

CDu =@(u) 1/(u^2 + 1);
OBu =@(u) CDu*[(b11.*u^2 + b12.*u + b13);
    (b21.*u^2 + b22.*u + b23);
    (b31.*u^2 + b32.*u + b33)];

%% curved surface tangential intersection
Guv_fun = @(u,v) (OBu(u) - OA).*v + OA;
Guv = Guv_fun(u,v);
f_surf = Fi(Guv(1),Guv(2),Guv(3));
% [N_f_surf,D_f_surf] = numden(f_surf);
[v_c,v_p] = coeffs(f_surf,v);
matlabFunction(v_c,'file','v_coeffs_T.m','Vars', {B,OA,C,u}); %translation
matlabFunction(v_c,'file','v_coeffs_O.m','Vars', {B,OA,C,u}); %orientation
for i = 1:size(v_c,2)
[tmp_val_1,tmp_val_2] = numden(v_c(i));
v_coeffs_new(i) = simplifyFraction(v_c(i)*(tmp_val_2));
end
% v_coeffs = matlabFunction(v_c,'file','v_coeffs.m','Vars', {u,b11,b12,b13,b21,b22,b23,b31,b32,b33,A1,A2,A3,c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,c15,c16,c17,c18,c19,c20,c21,c22,c23,c24,c25,c26,c27,c28}); 
%%
for i = 1:size(v_coeffs_new,2)
[u_v{i},u_power{i}] = coeffs(v_coeffs_new(i),u);
end
for i = 1:size(v_c,2)
[u_v{i},u_power{i}] = coeffs(v_c(i),u);
end
H4 = matlabFunction(u_v{1},'file','H4_T.m','Vars', {B,OA,C}); 
H3 = matlabFunction(u_v{2},'file','H3_T.m','Vars', {B,OA,C}); 
H2 = matlabFunction(u_v{3},'file','H2_T.m','Vars', {B,OA,C}); 
H1 = matlabFunction(u_v{4},'file','H1_T.m','Vars', {B,OA,C}); 
H0 = matlabFunction(u_v{5},'file','H0_T.m','Vars', {B,OA,C}); 
%% 4th degree surface
U4 = sym('u4%d',[1 9]);
U3 = sym('u3%d',[1 7]);
U2 = sym('u2%d',[1 5]);
U1 = sym('u1%d',[1 3]);
syms u0;
a_u(1) = U4*[u^8 u^7 u^6 u^5 u^4 u^3 u^2 u^1 u^0].';
b_u(2) = U3*[u^6 u^5 u^4 u^3 u^2 u^1 u^0].';
c_u(3) = U2*[u^4 u^3 u^2 u^1 u^0].';
d_u(4) = U1*[u^2 u^1 u^0].';
e_u(5) = u0;

u_fun = simplify(256*a_u^3*e_u^3 - 192*a_u^2*b_u*d_u*e_u^2 - 128*a_u^2*c_u^2*e_u^2 + 144*a_u^2*c_u*d_u^2*e_u + ...
    -27*a_u^2*d_u^4 + 144*a_u*b_u^2*c_u*e_u^2 - 6*a_u*b_u^2*d_u^2*e_u - 80*a_u*b_u*c_u^2*d_u*e_u + ...
    18*a_u*b_u*c_u*d_u^3 + 16*a_u*c_u^4*e_u - 4*a_u*c_u^3*d_u^2 - 27*b_u^4*e_u^2 + 18*b_u^3*c_u*d_u*e_u + ...
    -4*b_u^3*d_u^3 - 4*b_u^2*c_u^3*e_u + b_u^2*c_u^2*d_u^2);
[u_c,u_p] = coeffs(u_fun,u);
% matlabFunction(u_c,'file','u_coeffs.m','Vars', {U4,U3,U2,U1,u0});
%% 3rd degree surface
U3 = sym('u3%d',[1 7]);
U2 = sym('u2%d',[1 5]);
U1 = sym('u1%d',[1 3]);
syms u0;
a_u = U3*[u^6 u^5 u^4 u^3 u^2 u^1 u^0].'; %a
b_u = U2*[u^4 u^3 u^2 u^1 u^0].'; %b
c_u = U1*[u^2 u^1 u^0].'; %c
d_u= u0; %d

u_fun = simplify(a_u^2*c_u^2 - 4*a_u*c_u^3 - 4*b_u^3*d_u - 27*a_u^2*d_u^2 + 18*a_u*b_u*c_u*d_u);
[u_c,u_p] = coeffs(u_fun,u);
matlabFunction(u_c,'file','u_coeffs_3_deg.m','Vars', {U3,U2,U1,u0});
%% 2nd degree surface
U2 = sym('u2%d',[1 5]);
U1 = sym('u1%d',[1 3]);
syms u0;
a_u = U2*[u^4 u^3 u^2 u^1 u^0].'; %a
b_u = U1*[u^2 u^1 u^0].'; %b
c_u= u0; %c

u_fun = simplify(b_u^2 - 4*a_u*c_u);
[u_c,u_p] = coeffs(u_fun,u);
matlabFunction(u_c,'file','u_coeffs_2_deg.m','Vars', {U2,U1,u0});

%% line segment v.s implicit surface F(x,y,z)
OB = sym('B%d',[3,1]);
OA = sym('A%d',[3,1]);
ray = (OB-OA).*t + OA;
f_linear = Fi(ray(1),ray(2),ray(3));
[t_c,t_p] = coeffs(f_linear,t);
t_coeffs = matlabFunction(t_c,'file','s_coeffs_T.m','Vars', {OB,OA,C}); 

%% curve segment v.s implicit surface F(x,y,z)
CDu =@(u) 1/(u^2 + 1);
B = sym('b%d%d',[3,3]);
OBu =@(u) CDu*[(b11.*u^2 + b12.*u + b13);
    (b21.*u^2 + b22.*u + b23);
    (b31.*u^2 + b32.*u + b33)];
OB = OBu(u);
COB = simplifyFraction((u^2 + 1).^4*Fi(OB(1),OB(2),OB(3)));
[t_c,t_p] = coeffs(COB,u);
t_coeffs = matlabFunction(t_c,'file','s_coeffs_O.m','Vars', {B,C}); 

%% curve/line v.s implicit surface G(x,y,z)
G = sym('g%d',[10,1])
Gi = @(x,y,z) G(1)*x.^2 + G(2)*y.^2 + G(3)*z.^2 + G(4)*x.*y + ...
    G(5)*x.*z + G(6)*y.*z + G(7)*x + G(8)*y +  G(9)*z + G(10); 
p = sym('p%d%d',[3,5]);
PS = p*[t^4,t^3,t^2,t,1].';
G1 = Gi(PS(1),PS(2),PS(3));
[g_c,g_p] = coeffs(G1,t);
g_coeffs = matlabFunction(g_c,'file','g_coeffs.m','Vars', {p,G}); 


