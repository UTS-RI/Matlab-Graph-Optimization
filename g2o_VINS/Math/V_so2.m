function [ y ] = V_so2( x )

t =  norm(x);

if t < 0.0001
    y=eye(2);
else
     y= 1/t * [ sin(t)  cos(t)-1; 1-cos(t)  sin(t) ];
end


end

