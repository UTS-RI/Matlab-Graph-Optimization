function [ LocalCoordinate ] = UVD2LocalXYZ( uvd )

fx = 525.0;
fy = 525.0;
cx0 = 639.5;
cy0 = 479.5;

K_inv = [ 1/fx 0; 0  1/fy];
u=uvd(1); v=uvd(2); d=uvd(3); 

z=d;

xy= K_inv*z*[ u-cx0; v-cy0 ];



LocalCoordinate=[xy;z];


end

