function [ adX_inverse ] = ad_G_inverse( X )


R=X(1:3,1:3);
v_G=X(1:3,4);
p=X(1:3,5);


adX_inverse=[ R' zeros(3,6);  ...
      -R'*skew(v_G)  R'  zeros(3,3);...
      -R'*skew(p)    zeros(3,3)   R'  ];


end

