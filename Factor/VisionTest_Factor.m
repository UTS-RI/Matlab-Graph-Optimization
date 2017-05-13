function [  ErrorVector, Jacobian_Node  ] = VisionTest_Factor( Nodes_array , Measurement_values )

% Measurement_values is [u;v];

pose=Nodes_array{1};
f=Nodes_array{2};
R=pose(1:3,1:3);
p=pose(1:3,4);
Local= R'*( f - p );


fx = 525.0;
fy = 525.0;
cx0 = 639.5;
cy0 = 479.5;


d1 = diag( [ 1/fx, 1/fy] )*( Measurement_values - [cx0;cy0] );
d = [d1;1];
d = d/norm(d);

normLocal = norm(Local);
ErrorVector = Local/normLocal - d;


dh = 1/normLocal*eye(3)- Local*Local'/normLocal^3;


Jacobian_Node{1}=dh*[ R'*skew(f)      -R'    ]; 


Jacobian_Node{2}=dh*R';





end

