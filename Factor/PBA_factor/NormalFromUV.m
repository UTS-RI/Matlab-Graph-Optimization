function [ nuv ] = NormalFromUV( uv )
fx = 525.0;
fy = 525.0;
cx0 = 639.5;
cy0 = 479.5;
n = diag([1/fx, 1/fy]  )  *(uv - [cx0;cy0]);
m = [n;1];

nuv = m/norm(m);


end

