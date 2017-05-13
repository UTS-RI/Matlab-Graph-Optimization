function [ jrx_inverse ] = jacor_inverse( dx )
phi=sqrt(dx'*dx);

if phi<0.00001
    jrx_inverse=eye(3);
else 
    sp=skew(dx);
    jrx_inverse=eye(3)+0.5*sp+(1/phi^2 - (1+cos(phi))/(2*phi*sin(phi)))*sp*sp;
end

