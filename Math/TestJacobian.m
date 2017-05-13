

v1 = rand(9,1);
v2  = randn(9,1)*0.01;
v2 = ones(9,1)*0.01;

 [ V1 ] = SE23( v1 );
   V2   = SE23( v2);
   
   norm(V1*V2 - SE23( v1 + Jr_inverse(v1)*v2   ))
   norm(V1*V2 - SE23( v1 + v2   )) 

   
   norm(V2*V1  -SE23(  v1+  Jr_inverse(-v1)*v2    )    )