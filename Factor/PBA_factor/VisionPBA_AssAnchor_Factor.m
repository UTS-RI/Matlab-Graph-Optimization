function [ ErrorVector, Jacobian_Node ] = VisionPBA_AssAnchor_Factor( Nodes_array , Measurement_values )

pose_m=Nodes_array{1};  
pose_a=Nodes_array{2};  
feature=Nodes_array{3}; 

uv = Measurement_values.uv;
Tcb= Measurement_values.Tcb;


 
[X, J_m , J_a, J_i , J_f] =  PBA_Jacobian( pose_m, pose_a, pose_a, feature, Tcb);

ErrorVector= project(X)-uv;

if norm(ErrorVector)>2
   
    sss= 0;
    
end


Jacobian_Node{1} = dproject(X)*J_m;
Jacobian_Node{2} = dproject(X)*(J_a+J_i);
Jacobian_Node{3} = dproject(X)*J_f;


end

function project_X=project(X)

fx = 525.0;
fy = 525.0;
cx0 = 639.5;
cy0 = 479.5;
x=X(1);
y=X(2);
z=X(3);
project_X = [ fx*x/z; fy*y/z]+[cx0;cy0];

end

function dX=dproject(X)
x=X(1);
y=X(2);
z=X(3);
fx = 525.0;
fy = 525.0;
K = [ fx 0; 0  fy];
dX =  K* [z 0 -x; 0 z -y ]/(z^2);
end