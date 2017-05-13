function [ value, inf ] = rk4_manifold_mod( M, N , accbias_es, gyrobias_es)

% this function is used for preintegration via RK4 method in  Lie group SO(3) 

[~,m]=size(M.gyro);
h = M.timestep;

Y =  cell(1,m+1);

PP = cell(1,m+1);

Y{1}.J1=eye(3); Y{1}.j1_w=zeros(3,3);
Y{1}.J2=zeros(3,1); Y{1}.j2_w=zeros(3,3); Y{1}.j2_a=zeros(3,3);
Y{1}.J3=zeros(3,1); Y{1}.j3_w=zeros(3,3); Y{1}.j3_a=zeros(3,3);

PP{1}=zeros(9,9);

A = cell(1,m+1);
A{1} = eye(9);

for j=1:m
  yj = Y{j};
  Pj = PP{j};
    if j<m-1
  k1 = h*derivative_sub(j,M,yj, accbias_es, gyrobias_es);
  k2 = h*derivative_sub(j,M, specialadd( yj , k1/2 ), accbias_es, gyrobias_es );
  k3 = h*derivative_sub(j,M, specialadd( yj , k2/2 ), accbias_es, gyrobias_es );
  k4 = h*derivative_sub(j,M, specialadd( yj , k3 ), accbias_es, gyrobias_es );  
  Y{j+1}=specialadd( yj,   (k1 + 2*k2 + 2*k3 + k4)/6   );
  
  
  K1 = h*derivative_sub2(j,M,N,Pj, accbias_es, gyrobias_es);
  K2 = h*derivative_sub2(j,M,N, Pj+ K1/2 , accbias_es, gyrobias_es );
  K3 = h*derivative_sub2(j,M,N, Pj + K2/2 , accbias_es, gyrobias_es);
  K4 = h*derivative_sub2(j,M,N, Pj + K3 ,accbias_es, gyrobias_es );
  PP{j+1}= Pj +  (K1 + 2*K2 + 2*K3 + K4)/6 ;
  
    else 
         k=h*derivative_sub(j,M,yj,accbias_es, gyrobias_es);
         Y{j+1}=specialadd( yj,  k );   
         
         K=h*derivative_sub2(j,M,N, Pj,accbias_es, gyrobias_es);
         PP{j+1}=PP{j}+K;
    end  

    w=M.gyro(1:3,j);
    a=M.acc(1:3,j);
      K1 = h*derivative_sub3(j,M,N, A{j}, accbias_es, gyrobias_es);
      K2 = h*derivative_sub3(j,M,N, A{j}+ K1/2 , accbias_es, gyrobias_es );
      K3 = h*derivative_sub3(j,M,N, A{j} + K2/2 , accbias_es, gyrobias_es);
      K4 = h*derivative_sub3(j,M,N, A{j} + K3 ,accbias_es, gyrobias_es );
    
    A{j+1}=  A{j} + (K1 + 2*K2 + 2*K3 + K4)/6;
    
    
end

P= PP{m+1};

value=struct;
value.J1=Y{m+1}.J1; value.j1_w=Y{m+1}.j1_w; 

value.J2=Y{m+1}.J2; value.j2_w=Y{m+1}.j2_w; value.j2_a=Y{m+1}.j2_a;
value.J3=Y{m+1}.J3; value.j3_w=Y{m+1}.j3_w; value.j3_a=Y{m+1}.j3_a;

%% debug

J1 = value.J1; J2= value.J2; J3= value.J3;

J1'*skew(J2)
J1'*skew(J3)


%%%% debug

inf=inv(P);


end

function [kj]=derivative_sub(j, M, yj , accbias_es, gyrobias_es )
J1=yj.J1;
J2=yj.J2;
wt=M.gyro(1:3,j);
at=M.acc(1:3,j);


j1_w=yj.j1_w;
j2_w=yj.j2_w;
j2_a=yj.j2_a;
j3_w=yj.j3_w;
j3_a=yj.j3_a;



kj_dJ1= wt-gyrobias_es;
kj_dJ2= J1*(at-accbias_es);
kj_dJ3= J2;
kj_dJ1_w=-skew( wt- gyrobias_es )*j1_w-eye(3);
kj_dJ2_w= -J1*skew(at-accbias_es )*j1_w;
kj_dJ2_a= -J1;
kj_dJ3_w= j2_w;
kj_dJ3_a= j2_a;

kj=[ kj_dJ1 kj_dJ2 kj_dJ3 kj_dJ1_w kj_dJ2_a  kj_dJ2_w  kj_dJ3_a kj_dJ3_w ];

end


function [Kj]=derivative_sub2(j, M, N, Pj, accbias_es, gyrobias_es )
wt=M.gyro(1:3,j);
at=M.acc(1:3,j);

wt=wt-gyrobias_es;
at=at-accbias_es; 

A=[ -skew(wt) zeros(3,3)   zeros(3,3);...
       -skew(at) -skew(wt) zeros(3,3);...
       zeros(3,3)  eye(3)   -skew(wt) ]; 


Kj=A*Pj+Pj*A'+ N;

end


function [Kj]=derivative_sub3(j, M, N, Aj, accbias_es, gyrobias_es )
wt=M.gyro(1:3,j);
at=M.acc(1:3,j);

wt=wt-gyrobias_es;
at=at-accbias_es; 

A=[ -skew(wt) zeros(3,3)   zeros(3,3);...
       -skew(at) -skew(wt) zeros(3,3);...
       zeros(3,3)  eye(3)   -skew(wt) ]; 


Kj=A*Aj;

end



function [ z ]=specialadd( y , k )
z=y;
z.J1=y.J1*so3_exp( k(1:3,1) );
z.J2=y.J2+k(1:3,2);
z.J3=y.J3+k(1:3,3);

z.j1_w=y.j1_w+k(1:3, 4:6);
z.j2_a=y.j2_a+k(1:3, 7:9);
z.j2_w=y.j2_w+k(1:3, 10:12);

z.j3_a=y.j3_a+k(1:3, 13:15);
z.j3_w=y.j3_w+k(1:3, 16:18);


end
