function [ ErrorVector, Jacobian_Node ] = LinearVisionNoLandmark_Factor( Nodes_array , Measurement_values )

pose=Nodes_array{1};

f=Measurement_values.f;
uv=Measurement_values.uv;
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



K_inv = [ 1/fx 0; 0  1/fy];

A = K_inv*(uv - [cx0;cy0] );
AA = [ eye(2) -A];
 
ErrorVector=  AA*Local;



Jacobian_Node{1}= AA*[ R'*skew(f)  -R']; 




end

