function y=J_rota(X)
a=X(1);
b=X(2);
y=[1 0 -sin(b);...
    0  cos(a)  cos(b)*sin(a);...
    0 -sin(a)  cos(b)*cos(a)
    ];