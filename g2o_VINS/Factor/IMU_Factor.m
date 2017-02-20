function [ ErrorVector, Jacobian_Node ] = IMU_Factor( Nodes_array , Measurement_values )
g=[0;0;-9.8];
dt = Measurement_values.dt;

posei=Nodes_array{1}; vi=Nodes_array{2}; bi=Nodes_array{3};
Ri=posei(1:3,1:3); pi=posei(1:3,4);
posej=Nodes_array{4}; vj=Nodes_array{5}; bj=Nodes_array{6};
Rj=posej(1:3,1:3); pj=posej(1:3,4);
dbw=bi(4:6)-Measurement_values.bw;
dba=bi(1:3)-Measurement_values.ba;

%dbw=zeros(3,1);
%dba=zeros(3,1);

[ Rj_pre, vj_pre, pj_pre ]=Prediction_IMU(posei, vi, bi,  Measurement_values );
Xj_pre_inv =  [ Rj_pre'  -vj_pre  -Rj_pre'*pj_pre  ];
dR= Rj'*Rj_pre;
dX = [  dR  dR*vj_pre-vj    Rj'*(pj_pre - pj)];

ErrorVector_robot = se23_log( dX );
ErrorVector= [ ErrorVector_robot ];

%%%%% Notation
J1 = Measurement_values.J1;
j1_w = Measurement_values.j1_w;
J2 = Measurement_values.J2;
j2_w = Measurement_values.j2_w;
j2_a = Measurement_values.j2_a;
J3 = Measurement_values.J3;
j3_w = Measurement_values.j3_w;
j3_a = Measurement_values.j3_a;
%%%%% Notation


A = Jacobian_Lie_inverse(ErrorVector_robot)*adjoint(Xj_pre_inv);

%A = adjoint(Xj_pre_inv);

zero33=zeros(3,3);

% M = [eye(3) zero33 zero33  zero33   Ri*J1*Jacobian_Lie(-j1_w*dbw );...
%     skew(g)*dt  Ri   zero33  Ri*j2_a   Ri*j2_w; ...
% skew(g)*dt^2/2  Ri*dt  eye(3)  Ri*j3_a  Ri*j3_w ];
Vi = Ri*vi;

M = [eye(3) zero33 zero33  zero33   Ri*J1*j1_w;...
    skew(g)*dt  Ri   zero33  Ri*j2_a   Ri*j2_w+skew(Vi+g*dt+Ri*J2)*Ri*J1*j1_w; ...
skew(g)*dt^2/2  Ri*dt  eye(3)  Ri*j3_a  Ri*j3_w+skew(pi+1/2*g*dt^2+Vi*dt+Ri*J3)*Ri*J1*j1_w ];



N = [ eye(3) zeros(3,12); zero33 Rj zeros(3,9);  zeros(3,6) eye(3) zeros(3,6)  ];


Jacobian_whole = A*[M -N];

Jacobian_Node{1} =  [ Jacobian_whole(:, 1:3) Jacobian_whole(:, 7:9)];
Jacobian_Node{2} =  [ Jacobian_whole(:, 4:6) ];
Jacobian_Node{3} =  [ Jacobian_whole(:, 10:15)];

Jacobian_Node{4} =  [Jacobian_whole(:, 16: 18 ) Jacobian_whole(:, 22:24) ];
Jacobian_Node{5} =  [ Jacobian_whole(:, 19 : 21)];
Jacobian_Node{6} =  [ Jacobian_whole(:, 25:30) ];




end