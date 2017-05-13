function [ Jre ] = Jr( e )

e1=e(1:3);
e2=e(4:6);
e3=e(7:9);

  jr1= jaco_r( e1 );
  kr2= Kr(e1,e2);
  kr3= Kr(e1,e3);

  Jre= [jr1 zeros(3,6)  ;...
        kr2  jr1  zeros(3,3);...
        kr3  zeros(3,3)  jr1 ]; 

end

