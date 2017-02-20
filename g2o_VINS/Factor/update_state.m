function [ NodeValue ] = update_state( NodeTypeName, NodeValue, increment )

flag=0;

switch NodeTypeName 
    case 'Pose3'
        R=NodeValue(1:3,1:3); p=NodeValue(1:3,4); incre1=increment(1:3); incre2=increment(4:6);
        dR=so3_exp(incre1 );
        newR= dR*R; 
        newp= dR*p+ Jacobian_Lie(-incre1 )*incre2;
        NodeValue=[ newR  newp ];        
        flag=1;
    case 'Pose2' 
        R=NodeValue(1:2,1:2); p=NodeValue(1:2,3); incre1=increment(1); incre2=increment(2:3);
        dR=so2_exp(incre1);
        newR=dR*R;
  %     newp=dR*p+ V_so2( incre1  )*incre2;
        newp=p+incre2;
        NodeValue=[ newR  newp ];
                flag=1;
    case 'Landmark3'
        NodeValue=NodeValue+increment ;         flag=1;

    case 'Velocity3'
        NodeValue=NodeValue+increment ;           flag=1;

    case 'IMUbias'
        NodeValue=NodeValue+increment ;        flag=1;

    case 'Landmark2'
        NodeValue=NodeValue+increment ;        flag=1;
end

if flag==0
    sprintf('Warning: you have to define the update for the new Node in  update_state.m')
end







end

