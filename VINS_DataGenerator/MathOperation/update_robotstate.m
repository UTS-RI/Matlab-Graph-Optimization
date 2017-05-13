function [ x ] = update_robotstate( x, dx )

orientation=x(1:3,1:3);
velocity_l=x(1:3,4);  
position=x(1:3,5);


orientation=so3_exp(dx(1:3))*orientation;
velocity_l=velocity_l+dx(4:6);
position=so3_exp(dx(1:3))*position+jaco_r(-dx(1:3))*dx(7:9);
 

x(1:3,1:3)=orientation;
x(1:3,4)=velocity_l; 
x(1:3,5)=position;
end


