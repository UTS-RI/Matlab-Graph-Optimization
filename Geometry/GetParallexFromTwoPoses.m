function [ nAndtheta ] = GetParallexFromTwoPoses( pose1, pose2, uvArray )

R1 = pose1(1:3,1:3); p1=pose1(1:3,4);
R2 = pose2(1:3,1:3); p2=pose2(1:3,4);


fx = 525.0;
fy = 525.0;
cx0 = 639.5;
cy0 = 479.5;

uv1 = uvArray{1};
uv2 = uvArray{2};

d1 =  [[1/fx 0; 0 1/fy]*(uv1 - [cx0;cy0]); 1];
d1 = d1/norm(d1);

d2 =  [[1/fx 0; 0 1/fy]*(uv2 - [cx0;cy0]); 1];
d2 = d2/norm(d2);

n1 = R1*d1;  % in world frame
n2 = R2*d2;  % in world frame

n = d1;
theta =acos(n1'*n2);


nAndtheta = [n; theta];

end

