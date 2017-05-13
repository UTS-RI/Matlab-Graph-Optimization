function [fc, J_m , J_a, J_i , J_f] = PBA_Jacobian(  pose_m, pose_a, pose, feature, Tcb )
right =0; 
Rcb = Tcb(1:3,1:3);
tcb = Tcb(1:3,4);

Rbc = Rcb';
tbc = -Rcb'*tcb;



Rm=pose_m(1:3,1:3); pm=pose_m(1:3,4);
Ra=pose_a(1:3,1:3); pa=pose_a(1:3,4);
R=pose(1:3,1:3); p=pose(1:3,4);
n1=feature(1:3,1); theta = feature(4,1);

pmc = pm+Rm*tbc;
pac = pa+Ra*tbc;
pc  = p+ R*tbc;

a = pmc-pac; 
n_m = Rm*Rbc*n1;
b= pmc- pc;




N = cos(theta)*norm(skew(a)*n_m)*n_m +sin(theta)*( b-(a'*n_m)*n_m);
fc = Rcb* R'*N;

[ da_m, da_a,  da_f] = derivative_a( Rm, pm, Ra, pa, n1, theta , right, Rbc, tbc  ) ;
[ dnm_m, dnm_a, dnm_f] = derivative_nm( Rm, pm, Ra, pa, n1, theta , right, Rbc, tbc );
[ db_m, db_a,  db_f] = derivative_b( Rm, pm, Ra, pa, n1, theta , right, Rbc, tbc );

normCrossProduct=norm(skew(a)*n_m);

N1 = cos(theta)*( normCrossProduct*eye(3)+n_m*(skew(a)*n_m)'/normCrossProduct*skew(a))...
    -sin(theta)*(n_m*a'+(a'*n_m)*eye(3));

N2 = -cos(theta)*n_m*(skew(a)*n_m)'/normCrossProduct*skew(n_m)-sin(theta)*(n_m*n_m');

dN_m = N1*dnm_m+ N2*da_m+sin(theta)*db_m;
dN_a = N1*dnm_a+ N2*da_a+sin(theta)*db_a;
dN_f = N1*dnm_f+ N2*da_f+sin(theta)*db_f+...
        (b-(a'*n_m)*n_m)*[0  0 cos(theta)]+normCrossProduct*n_m*[0 0 -sin(theta)];
    
    
J_m =Rcb* R'*dN_m;
J_a = Rcb* R'*dN_a;
  peudo_f = N + sin(theta)*p;
J_f = Rcb*R'*dN_f;

if right ==1
    J_i = Rcb*[ R'*skew(peudo_f)  -sin(theta)*R']; 
elseif right ==0
    J_i = Rcb*[skew(Rbc*fc+sin(theta)*tbc)  -sin(theta)*eye(3)];
end
   


end

 



function [ da_m, da_a, da_f] = derivative_a( Rm, pm, Ra, pa, n1, theta , right, Rbc, tbc  ) 
if right == 1
da_m = [-skew(pm) eye(3)];
da_a = -[-skew(pa) eye(3)];
da_f = zeros(3,3);

else
    
da_m = [-Rm*skew(tbc) Rm];
da_a = [Ra*skew(tbc) -Ra];
da_f = zeros(3,3);

end

end

function [ dnm_m, dnm_a, dnm_f] = derivative_nm( Rm, pm, Ra, pa, n1, theta , right, Rbc, tbc ) 
if right == 1
dnm_m = [-skew(Rm*n1)   zeros(3,3)];
dnm_a = zeros(3,6);
dnm_f = -Rm*skew(n1)*[computeA(n1) zeros(3,1)];

else
dnm_m = [-Rm*skew(Rbc*n1)   zeros(3,3)];
dnm_a = zeros(3,6);
dnm_f = -Rm*Rbc*skew(n1)*[computeA(n1) zeros(3,1)];



end


end

function [ db_m, db_a,  db_f] = derivative_b( Rm, pm, Ra, pa, n1, theta , right, Rbc, tbc ) 
if right == 1
db_m = [-skew(pm) eye(3)];
db_a = zeros(3,6);
db_f = zeros(3,3);

else
    
db_m = [-Rm*skew(tbc) Rm];
db_a = zeros(3,6);
db_f = zeros(3,3);
    

end


end