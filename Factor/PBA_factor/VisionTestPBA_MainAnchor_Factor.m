function [ ErrorVector, Jacobian_Node ] = VisionTestPBA_MainAnchor_Factor( Nodes_array , Measurement_values )

%pose_m=Nodes_array{1};  
%pose_a=Nodes_array{2};  
feature=Nodes_array{1}; 

uv = Measurement_values.uv;
%Tcb= Measurement_values.Tcb;


n = feature(1:3,1);
A = computeA(n);


ErrorVector= n-NormalFromUV(uv);



%Jacobian_Node{1} = zeros(3,6);
%Jacobian_Node{2} = zeros(3,6);
Jacobian_Node{1} = [-skew(n)*A [0;0;0]];



% Jacobian_Node{1} = dDirectProject(X)*(J_m+J_i);
% Jacobian_Node{2} = dDirectProject(X)*J_a;
% Jacobian_Node{3} = dDirectProject(X)*J_f;


end
