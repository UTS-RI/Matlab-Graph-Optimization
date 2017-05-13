function [ y ] = SE23( EV )

y =   [so3_exp(EV(1:3)) jaco_r(-EV(1:3))*EV(4:6) jaco_r(-EV(1:3))*EV(7:9)];


y =  [y ; 0 0 0 1 0; 0 0 0 0 1]; 

end

