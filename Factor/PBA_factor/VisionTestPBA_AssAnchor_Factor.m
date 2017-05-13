function [ ErrorVector, Jacobian_Node ] = VisionTestPBA_AssAnchor_Factor( Nodes_array , Measurement_values )

pose_m=Nodes_array{1};  
pose_a=Nodes_array{2};  
feature=Nodes_array{3}; 

uv = Measurement_values.uv;
Tcb= Measurement_values.Tcb;


 
[X, J_m , J_a, J_i , J_f] =  PBA_Jacobian( pose_m, pose_a, pose_a, feature, Tcb);
%[X, J_m , J_a, J_f] =  PBA_JacobianAss( pose_m, pose_a,  feature, Tcb);


ErrorVector= DirectProject(X)-NormalFromUV(uv);

if norm(ErrorVector)>2
   
    sss= 0;
    
end


Jacobian_Node{1} = dDirectProject(X)*J_m;
Jacobian_Node{2} = dDirectProject(X)*(J_a+J_i);
%Jacobian_Node{2} = dDirectProject(X)*J_a;
Jacobian_Node{3} = dDirectProject(X)*J_f;


end