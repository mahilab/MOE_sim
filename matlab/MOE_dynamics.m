% Evan Pezent | evanpezent.com | epezent@rice.edu
% 02/04/2017

% =========================================================================
% This script computes the MOE dynamic equations symbolically using
% the Newton-Euler approach, and rearranges all terms in the form:
% Tau = M(Q)Q" + V(Q,Q') + G(Q) + B.*Q' + Fk.*sign(Q')

%% Define Symbolic Symbols
syms tau0 eta0 Jm0 q0 qd0 qdd0 m0 b0 fk0 ...
     tau1 eta1 Jm1 q1 qd1 qdd1 m1 b1 fk1 ...
     tau2 eta2 Jm2 q2 qd2 qdd2 m2 b2 fk2 ...
     tau3 eta3 Jm3 q3 qd3 qdd3 m3 b3 fk3 ...
     Pcx0 Pcy0 Pcz0   Icxx0 Icxy0 Icxz0 Icyy0 Icyz0 Iczz0 ...
     Pcx1 Pcy1 Pcz1   Icxx1 Icxy1 Icxz1 Icyy1 Icyz1 Iczz1 ...
     Pcx2 Pcy2 Pcz2   Icxx2 Icxy2 Icxz2 Icyy2 Icyz2 Iczz2 ...
     Pcx3 Pcy3 Pcz3   Icxx3 Icxy3 Icxz3 Icyy3 Icyz3 Iczz3 ...
     g

Tau = [tau0; tau1; tau2; tau3];

Eta = [eta0;eta1;eta2;eta3];
Jm = [Jm0;Jm1;Jm2;Jm3];

Q = [q0;q1;q2;q3];
Qd = [qd0;qd1;qd2;qd3];
Qdd = [qdd0;qdd1;qdd2;qdd3];

B = [b0;b1;b2;b3];
Fk = [fk0;fk1;fk2;fk3];

Pc0 = [Pcx0 Pcy0 Pcz0].';
Pc1 = [Pcx1 Pcy1 Pcz1].';
Pc2 = [Pcx2 Pcy2 Pcz2].';
Pc3 = [Pcx3 Pcy3 Pcz3].';

Ic0 = [Icxx0 -Icxy0 -Icxz0;
    -Icxy0 Icyy0 -Icyz0;
    -Icxz0 -Icyz0 Iczz0];

Ic1 = [Icxx1 -Icxy1 -Icxz1;
    -Icxy1 Icyy1 -Icyz1;
    -Icxz1 -Icyz1 Iczz1];

Ic2 = [Icxx2 -Icxy2 -Icxz2;
    -Icxy2 Icyy2 -Icyz2;
    -Icxz2 -Icyz2 Iczz2];

Ic3 = [Icxx3 -Icxy3 -Icxz3;
    -Icxy3 Icyy3 -Icyz3;
    -Icxz3 -Icyz3 Iczz3];

%% Forward Kinematics
DH_table = [    0     0 0      q0;
            -0.15 -pi/2 0 q1-pi/2;
                0  pi/2 0      q2;
                0  pi/2 0      q3];

[~,T_array] = dh2tf(DH_table);

%% Newton-Euler Dynamics
m = [m0;m1;m2;m3];
Pc = {Pc0 Pc1 Pc2 Pc3};
Ic = {Ic0 Ic1 Ic2 Ic3};
g0 = [g; 0; 0];
MVG = dynamics_newtonian(m,Pc,Ic,T_array,Qd,Qdd,g0);
MVG = simplify(expand(MVG));

%% Separate MVG into M, V, and G
[M,V,G] = separate_mvg(MVG,Qdd,g);

%% Write M, V, G to file
if ~exist('Equations', 'dir')
    mkdir('Equations')
end
ccode(M,'File','Equations/M.txt')
ccode(V,'File','Equations/V.txt')
ccode(G,'File','Equations/G.txt')
%% Get Equation of Motion
EOM = Tau == M*Qdd + Jm.*Eta.^2.*Qdd + V + G + B.*Qd + Fk.*tanh(10 * Qd);

%% Solved for acclerations ( i.e inv(M) * (Tau - V - G) )
Qdd_solved = (M + diag(Jm.*Eta.^2))\(Tau - V - G - B.*Qd - Fk.*tanh(10 * Qd));

%% Numerical Evaluation NOT USING THIS YET
openWrist = OpenWristInit();

m0 = openWrist.PS.m;
m1 = openWrist.FE.m;
m2 = openWrist.RU.m;

eta0 = openWrist.PS.eta;
eta1 = openWrist.FE.eta;
eta2 = openWrist.RU.eta;

Jm0 = openWrist.PS.motor.J;
Jm1 = openWrist.FE.motor.J;
Jm2 = openWrist.RU.motor.J;

b0 = openWrist.PS.B;
b1 = openWrist.FE.B;
b2 = openWrist.RU.B;

fk0 = openWrist.PS.fk;
fk1 = openWrist.FE.fk;
fk2 = openWrist.RU.fk;

Pc0x = openWrist.PS.Xc;
Pc0y = openWrist.PS.Yc;
Pc0z = openWrist.PS.Zc;

Pc1x = openWrist.FE.Xc;
Pc1y = openWrist.FE.Yc;
Pc1z = openWrist.FE.Zc;

Pc2x = openWrist.RU.Xc;
Pc2y = openWrist.RU.Yc;
Pc2z = openWrist.RU.Zc;

Ic0xx = openWrist.PS.Icxx;
Ic0xy = openWrist.PS.Icxy;
Ic0xz = openWrist.PS.Icxz;
Ic0yy = openWrist.PS.Icyy;
Ic0yz = openWrist.PS.Icyz;
Ic0zz = openWrist.PS.Iczz;

Ic1xx = openWrist.FE.Icxx;
Ic1xy = openWrist.FE.Icxy;
Ic1xz = openWrist.FE.Icxz;
Ic1yy = openWrist.FE.Icyy;
Ic1yz = openWrist.FE.Icyz;
Ic1zz = openWrist.FE.Iczz;

Ic2xx = openWrist.RU.Icxx;
Ic2xy = openWrist.RU.Icxy;
Ic2xz = openWrist.RU.Icxz;
Ic2yy = openWrist.RU.Icyy;
Ic2yz = openWrist.RU.Icyz;
Ic2zz = openWrist.RU.Iczz;

g = openWrist.g;

M_num = simplify(expand(eval(M)));
V_num = simplify(expand(eval(V)));
G_num = simplify(expand(eval(G)));
B_num = eval(B);
Fk_num = eval(Fk);
Jm_num = eval(Jm);
Eta_num = eval(Eta);

%% Numerical Model
EOM_num = M_num*Qdd + Jm_num.*Eta_num.^2.*Qdd + V_num + G_num + B_num.*Qd + Fk_num.*tanh(10*Qd) - Tau;

%% Solve q1dd HAVE NOT TOUCHED ANYTHING BELOW HERE YET
LHS1 = simplify(expand(solve(EOM_num(1) == 0, q2dd)));
MHS1 = simplify(expand(solve(EOM_num(2) == 0, q2dd)));
RHS1 = simplify(expand(solve(EOM_num(3) == 0, q2dd)));
LHS2 = (solve(LHS1==MHS1,q3dd));
RHS2 = (solve(MHS1==RHS1,q3dd));
q1dd_solved = solve(LHS2==RHS2,q1dd);
%% Solve q2dd
LHS1 = simplify(expand(solve(EOM_num(1) == 0, q3dd)));
MHS1 = simplify(expand(solve(EOM_num(2) == 0, q3dd)));
RHS1 = simplify(expand(solve(EOM_num(3) == 0, q3dd)));
LHS2 = (solve(LHS1==MHS1,q1dd));
RHS2 = (solve(MHS1==RHS1,q1dd));
q2dd_solved = solve(LHS2==RHS2,q2dd);
%% Solve q3dd
LHS1 = simplify(expand(solve(EOM_num(1) == 0, q1dd)));
MHS1 = simplify(expand(solve(EOM_num(2) == 0, q1dd)));
RHS1 = simplify(expand(solve(EOM_num(3) == 0, q1dd)));
LHS2 = (solve(LHS1==MHS1,q2dd));
RHS2 = (solve(MHS1==RHS1,q2dd));
q3dd_solved = solve(LHS2==RHS2,q3dd);
