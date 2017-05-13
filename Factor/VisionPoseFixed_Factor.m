function [ ErrorVector, Jacobian_Node ] = VisionPoseFixed_Factor( Nodes_array , Measurement_values )


pose =  Measurement_values.pose;
uv   =  Measurement_values.uv;


f=Nodes_array{1};
R=pose(1:3,1:3);
p=pose(1:3,4);
Local= R'*( f - p );

fx = 525.0;
fy = 525.0;
cx0 = 639.5;
cy0 = 479.5;

x= Local(1);
y= Local(2);
z= Local(3);

K = [ fx 0; 0  fy];

h =  K*[x/z; y/z]+ [cx0;cy0];
ErrorVector= h - [uv(1);uv(2)];

dh =  K* [z 0 -x; 0 z -y ]/(z^2);

Jacobian_Node{1}=dh*R';



end

