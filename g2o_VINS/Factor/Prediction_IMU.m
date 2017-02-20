function [ Rj_pre, vj_pre, pj_pre ] = Prediction_IMU( posei, vi, bi,  Measurement_values )

%%%%%%% Notation Definition

dbw=bi(4:6)-Measurement_values.bw;
dba=bi(1:3)-Measurement_values.ba;

% dbw=[0;0;0];
% dba=[0;0;0];


J1 = Measurement_values.J1;
j1_w = Measurement_values.j1_w;

J2 = Measurement_values.J2;
j2_w = Measurement_values.j2_w;
j2_a = Measurement_values.j2_a;


J3 = Measurement_values.J3;
j3_w = Measurement_values.j3_w;
j3_a = Measurement_values.j3_a;


dt=Measurement_values.dt;
g=[0;0;-9.8];

Ri = posei(1:3,1:3);
pi = posei(1:3,4);


%%%%%%%% Prediction 

Rj_pre= Ri*J1*so3_exp( j1_w * dbw  );

Vj_pre= Ri*vi + dt*g+ Ri*( J2+ j2_w* dbw + j2_a*dba  );

vj_pre= Rj_pre'*  Vj_pre;

pj_pre= pi + dt^2/2*g + dt*Ri*vi+Ri*(J3+ j3_w* dbw + j3_a*dba );





end

