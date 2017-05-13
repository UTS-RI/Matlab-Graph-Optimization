function [ Jreinvserse ] = Jr_inverse( e )

e1=e(1:3);
e2=e(4:6);
e3=e(7:9);

j1=jacor_inverse(e1);
Qr2=Kr( e1, e2 );
Qr3=Kr( e1, e3 );

Jreinvserse=[  j1   zeros(3,6)   ;...
                -j1*Qr2*j1  j1  zeros(3,3);...
               -j1*Qr3*j1   zeros(3,3)   j1  ];






end

