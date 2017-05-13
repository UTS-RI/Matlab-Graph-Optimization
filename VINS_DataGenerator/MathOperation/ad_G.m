function [ adX ] = ad_G( X )


R=X(1:3,1:3);
v_G=X(1:3,4);
p=X(1:3,5);


adX=[ R zeros(3,6);  ...
      skew(v_G)*R  R  zeros(3,3);...
      skew(p)*R    zeros(3,3)   R  ];


end

