function [ Y ] = Jacobian_Lie_inverse( x )

sizex=size(x,1);

switch sizex
    case 3
          x1=x(1:3); Y= jacor_inverse( x1 );
    case 6
          x1=x(1:3); x2=x(4:6); j1=jacor_inverse( x1 ); Q=Kr(x1,x2);     Y= [ j1  zeros(3,3); -j1*Q*j1 j1 ];         
    case 9    
          x1=x(1:3); x2=x(4:6);x3=x(7:9); 
          j1=jacor_inverse( x1 ); 
          Q2=Kr(x1,x2);
          Q3=Kr(x1,x3); 
          Y= [ j1  zeros(3,6); -j1*Q2*j1 j1 zeros(3,3);   -j1*Q3*j1   zeros(3,3)  j1];
end


end

