function [ UV_measurement  ] = GenerateUV_randn( pose, landmark )

sigma=1/sqrt(2);
R=pose(1:3,1:3);
p=pose(1:3,4);
f=landmark;



Local= R'*( f - p );


fx = 525.0;
fy = 525.0;
cx0 = 639.5;
cy0 = 479.5;

x= Local(1);
y= Local(2);
z= Local(3);

K = [ fx 0; 0  fy];

noise = sqrt(2)/2* sigma*randn(2,1);
UV_measurement =  K*[x/z; y/z]+ [cx0;cy0] + noise;

end

