clc


load matlab.mat;
load matlab2.mat;

g = [0;0;-9.8]; dt =0.4;
Pose0 = RobotState{1, 1}.pose; Pose1 = RobotState{1, 2}.pose; 
R0 = Pose0(1:3,1:3); P0= Pose0(1:3,4); R1 = Pose1(1:3,1:3); P1= Pose1(1:3,4);
v0 = RobotState{1, 1}.velocity; v1 = RobotState{1, 2}.velocity; 
dR = R0'*R1;
dV = R0'*( R1*v1 - R0*v0 - g*dt );
dP = R0'*( P1- P0 - R0*v0*dt - 0.5*g*dt^2 );


[ value, inf ] = rk4_manifold_mod( M, N , accbias_es, gyrobias_es);
[ value2, inf ] = rk4_manifold_mod2( M, N , accbias_es, gyrobias_es);
[ value3, inf ] = rk4_manifold_mod3( M, N , accbias_es, gyrobias_es);
[ value4, inf ] = rk4_manifold_mod4( M, N , accbias_es, gyrobias_es);
A0 =[dR dV dP]
A1= [value.J1 value.J2 value.J3]
A2= [value2.J1 value2.J2 value2.J3]
A3= [value3.J1 value3.J2 value3.J3]
A4= [value4.J1 value4.J2 value4.J3]
