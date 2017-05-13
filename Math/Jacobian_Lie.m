function [ Y ] = Jacobian_Lie( x )


sizex=size(x,1);

switch sizex
    case 3
          x1=x(1:3); Y= jaco_r( x1 );
    case 6
          x1=x(1:3); x2=x(4:6); Y= [ jaco_r( x1 ) zeros(3,3);  Kr(x1,x2)     jaco_r( x1 )  ]  ;        
    case 9    
          x1=x(1:3); x2=x(4:6); x3=x(7:9); Y= [ jaco_r( x1 ) zeros(3,6);  Kr(x1,x2)     jaco_r( x1 ) zeros(3,3); Kr(x1,x3)    zeros(3,3)      jaco_r( x1 )       ]  ;
end



end

