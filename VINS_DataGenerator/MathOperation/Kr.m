function [ Kr_value ] = Kr( theta1, theta2 )


Kr_value=Kl(-theta1,-theta2);




end


function Kl_value=Kl(x1,x2)


theta=norm(x1);


Kl_value=1/2*skew(x2)+(theta-sin(theta))/theta^3*(skew(x1)*skew(x2)+skew(x2)*skew(x1)+skew(x1)*skew(x2)*skew(x1))...
        -(1-theta^2/2-cos(theta))/theta^4*(skew(x1)*skew(x1)*skew(x2)+skew(x2)*skew(x1)*skew(x1)-3*skew(x1)*skew(x2)*skew(x1) )...
        -1/2*( (1-theta^2/2-cos(theta))/theta^4-3*(theta-sin(theta)-theta^3/6)/theta^5)*(skew(x1)*skew(x2)*skew(x1)*skew(x1)+skew(x1)*skew(x1)*skew(x2)*skew(x1)); 





end