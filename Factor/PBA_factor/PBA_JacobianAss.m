function [fc, J_m , J_a , J_f] = PBA_JacobianAss(  pose_m, pose_a, feature, Tcb )
right =0; 
Rcb = Tcb(1:3,1:3);
tcb = Tcb(1:3,4);

Rbc = Rcb';
tbc = -Rcb'*tcb;



Rm=pose_m(1:3,1:3); pm=pose_m(1:3,4);
Ra=pose_a(1:3,1:3); pa=pose_a(1:3,4);
n1=feature(1:3,1); theta = feature(4,1);

pmc = pm+Rm*tbc;
pac = pa+Ra*tbc;

a = pmc-pac; 
nm = Rm*Rbc*n1;

anmnorm = norm(skew(a)*nm);

N = cos(theta)*anmnorm*nm+sin(theta)*(a- (a'*nm)*nm);

fc = Rcb*Ra'*N;

J1 = cos(theta)*(anmnorm*eye(3)+nm*(skew(a)*nm)'/anmnorm*skew(a))...
    -sin(theta)*(nm*a'+(a'*nm)*eye(3));

J2 = -cos(theta)*nm*(skew(a)*nm)'/anmnorm*skew(nm)...
    -sin(theta)*nm*nm'+sin(theta)*eye(3);

J3 = (a-(a'*nm)*nm)*[0 0 cos(theta)]+anmnorm*nm*[0 0 -sin(theta)];


[ da_m, da_a,  da_f] = derivative_a( Rm, pm, Ra, pa, n1, theta , right, Rbc, tbc  ) ;
[ dnm_m, dnm_a, dnm_f] = derivative_nm( Rm, pm, Ra, pa, n1, theta , right, Rbc, tbc );


d_N_m = J1*dnm_m + J2*da_m; 
d_N_a = J1*dnm_a + J2*da_a;
d_N_f = J1*dnm_f + J2*da_f+J3;



J_m = Rbc'*Ra'*d_N_m;
J_a = Rbc'*(Ra'*d_N_a+ [skew(Ra'*N) zeros(3,3)] );
J_f = Rbc'* Ra'*d_N_f;


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
