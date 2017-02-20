clc
clear
load VINS4
H =zeros(3,3);


    f = [ 0 ; 0; 0 ]; 
  %  T = RobotState{1, i}.pose;
  T1 = [eye(3)   [0;0; -2]];
 % T2 = [eye(3)   [0;0;  2]];
    T2 = [ [ 0 1 0; 0 0 1; 1 0 0   ]   [0;-2;  0]];

    %T(1:3,1:3) = 
    Nodes_array{1}= T1;
    Nodes_array{2}=f;
    Measurement_values=[1;2];
[ ErrorVector, Jacobian_Node ] = Vision_Factor( Nodes_array , Measurement_values );

    J2=Jacobian_Node{2};
        Nodes_array{1}= T2;

[ ErrorVector2, Jacobian_Node2 ] = Vision_Factor( Nodes_array , Measurement_values );

    H1 = J2'*J2;
    H2 = Jacobian_Node2{2}'*Jacobian_Node2{2};
    
    HH=H1+H2

cond(HH)


