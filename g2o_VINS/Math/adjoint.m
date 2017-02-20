function [ y ] = adjoint( X )

y=0;
sizeX=size(X,2);


switch sizeX
    case 3   
           R=X(1:3,1:3);   y=R;
    case 4    
           R=X(1:3,1:3); p=X(1:3,4);  y=[ R zeros(3,3); skew(p)*R  R ];
    case 5
           R=X(1:3,1:3); v=X(1:3,4); p=X(1:3,5); y=[ R zeros(3,6); skew(v)*R  R zeros(3,3); skew(p)*R  zeros(3,3)   R  ];
end


end

