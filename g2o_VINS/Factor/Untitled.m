clc
clear


F =Jacobian_Node{1}; dx = rand(6,1); 
E =Jacobian_Node{2}   ;     dy =rand(3,1);
C =ErrorVector;

nullE=null(E)

 norm(F*dx+E*dy + ErrorVector,2) 

 norm( nullE*( F*dx+E*dy + C ) ,2 )