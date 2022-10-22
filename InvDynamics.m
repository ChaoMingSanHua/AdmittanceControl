function Tau = InvDynamics(q, dq, ddq, g, DH_para)

Tau = zeros(6, 1);

Alpha = DH_para.Alpha;
A = DH_para.A;
D = DH_para.D;
Theta = DH_para.Theta;

m1 = 0.1; x1 = 0.2; y1 = 0.3; z1 = 0.4; xx1 = 0.5; xy1 = 0.6; xz1 = 0.7; yy1 = 0.8; yz1 = 0.9; zz1 = 1.0;
m2 = 1.1; x2 = 1.2; y2 = 1.3; z2 = 1.4; xx2 = 1.5; xy2 = 1.6; xz2 = 1.7; yy2 = 1.8; yz2 = 1.9; zz2 = 2.0;
m3 = 2.1; x3 = 2.2; y3 = 2.3; z3 = 2.4; xx3 = 2.5; xy3 = 2.6; xz3 = 2.7; yy3 = 2.8; yz3 = 2.9; zz3 = 3.0;
m4 = 3.1; x4 = 3.2; y4 = 3.3; z4 = 3.4; xx4 = 3.5; xy4 = 3.6; xz4 = 3.7; yy4 = 3.8; yz4 = 3.9; zz4 = 4.0;
m5 = 4.1; x5 = 4.2; y5 = 4.3; z5 = 4.4; xx5 = 4.5; xy5 = 4.6; xz5 = 4.7; yy5 = 4.8; yz5 = 4.9; zz5 = 5.0;
m6 = 5.1; x6 = 5.2; y6 = 5.3; z6 = 5.4; xx6 = 5.5; xy6 = 5.6; xz6 = 5.7; yy6 = 5.8; yz6 = 5.9; zz6 = 6.0;

mx1 = m1 * x1; my1 = m1 * y1; mz1 = m1 * z1; Ixx1 = xx1+m1*(y1^2+z1^2); Ixy1 = xy1-m1*x1*y1; Ixz1 = xz1-m1*x1*y1; Iyy1 = yy1+m1*(x1^2+z1^2); Iyz1 = yz1-m1*y1*z1; Izz1 = zz1+m1*(x1^2+y1^2);
mx2 = m2 * x2; my2 = m2 * y2; mz2 = m2 * z2; Ixx2 = xx2+m2*(y2^2+z2^2); Ixy2 = xy2-m2*x2*y2; Ixz2 = xz2-m2*x2*y2; Iyy2 = yy2+m2*(x2^2+z2^2); Iyz2 = yz2-m2*y2*z2; Izz2 = zz2+m2*(x2^2+y2^2);
mx3 = m3 * x3; my3 = m3 * y3; mz3 = m3 * z3; Ixx3 = xx3+m3*(y3^2+z3^2); Ixy3 = xy3-m3*x3*y3; Ixz3 = xz3-m3*x3*y3; Iyy3 = yy3+m3*(x3^2+z3^2); Iyz3 = yz3-m3*y3*z3; Izz3 = zz3+m3*(x3^2+y3^2);
mx4 = m4 * x4; my4 = m4 * y4; mz4 = m4 * z4; Ixx4 = xx4+m4*(y4^2+z4^2); Ixy4 = xy4-m4*x4*y4; Ixz4 = xz4-m4*x4*y4; Iyy4 = yy4+m4*(x4^2+z4^2); Iyz4 = yz4-m4*y4*z4; Izz4 = zz4+m4*(x4^2+y4^2);
mx5 = m5 * x5; my5 = m5 * y5; mz5 = m5 * z5; Ixx5 = xx5+m5*(y5^2+z5^2); Ixy5 = xy5-m5*x5*y5; Ixz5 = xz5-m5*x5*y5; Iyy5 = yy5+m5*(x5^2+z5^2); Iyz5 = yz5-m5*y5*z5; Izz5 = zz5+m5*(x5^2+y5^2);
mx6 = m6 * x6; my6 = m6 * y6; mz6 = m6 * z6; Ixx6 = xx6+m6*(y6^2+z6^2); Ixy6 = xy6-m6*x6*y6; Ixz6 = xz6-m6*x6*y6; Iyy6 = yy6+m6*(x6^2+z6^2); Iyz6 = yz6-m6*y6*z6; Izz6 = zz6+m6*(x6^2+y6^2);

jointInertia1 = 0; jointInertia2 = 0; jointInertia3 = 0; jointInertia4 = 0; jointInertia5 = 0; jointInertia6 = 0;
JointInertia = [jointInertia1, jointInertia2, jointInertia3, jointInertia4, jointInertia5, jointInertia6];

Mass = [m1, m2, m3, m4, m5, m6];
PcmVector1 = [mx1; my1; mz1];
PcmVector2 = [mx2; my2; mz2];
PcmVector3 = [mx3; my3; mz3];
PcmVector4 = [mx4; my4; mz4];
PcmVector5 = [mx5; my5; mz5];
PcmVector6 = [mx6; my6; mz6];
PcmVecotr = [PcmVector1, PcmVector2, PcmVector3, PcmVector4, PcmVector5, PcmVector6];
InertiaMatrix1 = [Ixx1, Ixy1, Ixz1; Ixy1, Iyy1, Iyz1; Ixz1, Iyz1, Izz1];
InertiaMatrix2 = [Ixx2, Ixy2, Ixz2; Ixy2, Iyy2, Iyz2; Ixz2, Iyz2, Izz2];
InertiaMatrix3 = [Ixx3, Ixy3, Ixz3; Ixy3, Iyy3, Iyz3; Ixz3, Iyz3, Izz3];
InertiaMatrix4 = [Ixx4, Ixy4, Ixz4; Ixy4, Iyy4, Iyz4; Ixz4, Iyz4, Izz4];
InertiaMatrix5 = [Ixx5, Ixy5, Ixz5; Ixy5, Iyy5, Iyz5; Ixz5, Iyz5, Izz5];
InertiaMatrix6 = [Ixx6, Ixy6, Ixz6; Ixy6, Iyy6, Iyz6; Ixz6, Iyz6, Izz6];

InertiaMatrix = zeros(3, 3, 6);
InertiaMatrix(:, :, 1) = InertiaMatrix1;
InertiaMatrix(:, :, 2) = InertiaMatrix2;
InertiaMatrix(:, :, 3) = InertiaMatrix3;
InertiaMatrix(:, :, 4) = InertiaMatrix4;
InertiaMatrix(:, :, 5) = InertiaMatrix5;
InertiaMatrix(:, :, 6) = InertiaMatrix6;

Omega = zeros(3, 6);
DOmega = zeros(3, 6);
Dv = zeros(3, 6);
f = zeros(3, 6);
n = zeros(3, 6);
R = zeros(3, 3, 6);
Rtranspose = zeros(3, 3, 6);
P = zeros(3, 6);

Omega0 = zeros(3, 1);
DOmega0 = zeros(3, 1);
Dv0 = zeros(3, 1);
Dv0(3) = g;
Z = [0; 0; 1];

for i = 1:6
    [R(:, :, i), P(:, i)] = GetTransforMat(Alpha(i), A(i), D(i), Theta(i), q(i));
    Rtranspose(:, :, i) = R(:, :, i)';
end

%% 正向迭代
for i = 1:6
    if i == 1
        Omega(:, i) = Rtranspose(:, :, i) * Omega0 + dq(i) * Z;
        DOmega(:, i) = Rtranspose(:, :, i) * DOmega0 + ddq(i) * Z + cross((Rtranspose(:, :, i) * Omega0), (dq(i) * Z));
        Dv(:, i) = Rtranspose(:, :, i) * ( cross(DOmega0, P(:, i)) + cross( Omega0, cross(Omega0, P(:, i))) + Dv0);
    else
        Omega(:, i) = Rtranspose(:, :, i) * Omega(:, i-1) + dq(i) * Z;
        DOmega(:, i) = Rtranspose(:, :, i) * DOmega(:, i-1) + ddq(i) * Z + cross((Rtranspose(:, :, i) * Omega(:, i-1)), (dq(i) * Z));
        Dv(:, i) = Rtranspose(:, :, i) * ( cross(DOmega(:, i-1), P(:, i)) + cross( Omega(:, i-1), cross(Omega(:, i-1), P(:, i))) + Dv(:, i-1));
    end
end

%% 逆向迭代
for i = 6:-1:1
    if i == 6
        f(:, i) = Mass(i) * Dv(:, i) + cross(DOmega(:, i), PcmVecotr(:, i)) + cross(Omega(:, i), cross(Omega(:, i), PcmVecotr(:, i)));
        n(:, i) =  - cross(Dv(:, i), PcmVecotr(:, i)) + InertiaMatrix(:, :, i) * DOmega(:, i) + cross(Omega(:, i), InertiaMatrix(:, :, i) * Omega(:, i));
    else
        f(:, i) =  R(:, :, i+1) * f(:, i+1) + Mass(i) * Dv(:, i) + cross(DOmega(:, i), PcmVecotr(:, i)) + cross(Omega(:, i), cross(Omega(:, i), PcmVecotr(:, i)));
        n(:, i) =  R(:, :, i+1) * n(:, i+1) + cross(P(:, i+1), R(:, :, i+1)*f(:, i+1)) - cross(Dv(:, i), PcmVecotr(:, i)) + InertiaMatrix(:, :, i) * DOmega(:, i) + cross(Omega(:, i), InertiaMatrix(:, :, i) * Omega(:, i));
    end
end

%% 关节惯性力
JointInertiaForce = zeros(1, 6);
for i = 1:6
    JointInertiaForce(i) = JointInertia(i) * ddq(i);
end

%% 关节力
 for i = 1:6
     Tau(i) = n(3, i) + JointInertiaForce(i);
 end
 