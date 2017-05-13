function [ Jvision_ri, Jvision_fk, ErrorVector_vision_ri_fk ] = Factor_Vision( xi, fk, Vision_ik )

R=xi(1:3,1:3);
p=xi(1:3,5);

hL =R'*(fk-p);
hL1=hL(1);
hL2=hL(3);
hL3=hL(3);
ErrorVector_vision_ri_fk=[ hL(1)/hL(3)-Vision_ik(1); hL(2)/hL(3)-Vision_ik(2) ]; 

H= 1/(hL3^2) * [ hL3 0 -hL1; 0 hL3 -hL2    ];

hi= [R'*skew(fk)  zeros(3,3)  -R'];
Jvision_ri=H*hi;

hk= R';
Jvision_fk= H*hk;


end

