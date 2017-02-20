function [ ErrorVector, Jacobian_Node ] = VisionNoLandmark_Factor( Nodes_array , Measurement_values )

pose=Nodes_array{1};

f=Measurement_values.f;
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
ErrorVector= h - Measurement_values.uv;

dh =  K* [z 0 -x; 0 z -y ]/(z^2);

Jacobian_Node{1}=dh*[ R'*skew(f)      -R'    ]; 
%Jacobian_Node{2}=dh*R';



end

